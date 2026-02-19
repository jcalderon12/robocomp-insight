
#ifndef DSRDECODER_H
#define DSRDECODER_H

#include "DSRTypeTrait.h"
#include <vector>
#include <map>
#include <optional>
#include <string>
#include <sstream>
#include <iostream>

struct DSREvent {
    uint64_t timestamp; // timestamp
    std::string modification_type; // K, MN, MNA, ME, MEA, DN, DE

    std::optional<uint64_t> node_id;        // node id
    std::optional<uint64_t> edge_from_id;   // edge from id
    std::optional<uint64_t> edge_to_id;     // edge to id
    std::optional<std::string> type;        // node/edge type
    std::optional<std::string> node_name;   // node name

    std::map<std::string, DSR::Attribute> attributes; // node/edge attributes

    std::vector<DSR::Node> nodes; // node vector for keyframe
    std::vector<DSR::Edge> edges; // edge vector for keyframe
};


class DSRDecoder{
public:
    std::unique_ptr<DSREvent> decode(const std::string &raw_message) {
        auto mod = std::make_unique<DSREvent>();

        // Take timestamp and modification type
        auto [timestamp, mod_type, content] = split_header(raw_message);
        mod->timestamp = timestamp;
        mod->modification_type = mod_type;
        
        // Parse content by type
        if (mod_type == DSRSpecialChars::K)
            parse_keyframe(content, *mod);
        else
            parse_modification(content, *mod);
            
        return mod;
    }

private:
    std::tuple<uint64_t, std::string, std::string> split_header(const std::string &msg) {
        // Find first character -> first hash
        size_t first_hash = msg.find(DSRSpecialChars::SLOT);
        // Find next character from previous -> second hash
        size_t second_hash = msg.find(DSRSpecialChars::SLOT, first_hash + 1);

        // Make timestamp from 0 to first hash
        uint64_t ts = std::stoull(msg.substr(0, first_hash));
        // Make modification type from first hash to second hash 
        std::string type = msg.substr(first_hash + 1, second_hash - first_hash - 1);
        // Make content from second hash to end
        std::string content = msg.substr(second_hash + 1);

        return {ts, type, content};
    }

    void parse_modification(const std::string &content, DSREvent &mod) {
        // Get other attributes
        parse_attributes(content, mod.attributes);
        
        // Node common attributes
        if (mod.modification_type == DSRSpecialChars::MN || 
            mod.modification_type == DSRSpecialChars::MNA ||
            mod.modification_type == DSRSpecialChars::DN) {
            
            // Get node id
            if (mod.attributes.count(DSRAttributeNames::ID)) 
                mod.node_id = std::get<uint64_t>(mod.attributes[DSRAttributeNames::ID].value());

            // Get node type and node name
            if (mod.modification_type == DSRSpecialChars::MN){
                if (mod.attributes.count(DSRAttributeNames::TYPE))
                    mod.type = std::get<std::string>(mod.attributes[DSRAttributeNames::TYPE].value());
                if (mod.attributes.count(DSRAttributeNames::NODE_NAME))
                    mod.node_name = std::get<std::string>(mod.attributes[DSRAttributeNames::NODE_NAME].value());
            }
        }

        // Edge common attributes
        else if (mod.modification_type == DSRSpecialChars::ME ||
                mod.modification_type == DSRSpecialChars::MEA ||
                mod.modification_type == DSRSpecialChars::DE) {
                
                // Get edge id's (from / to)     
                if (mod.attributes.count(DSRAttributeNames::IDF))
                    mod.edge_from_id = std::get<uint64_t>(mod.attributes[DSRAttributeNames::IDF].value());
                if (mod.attributes.count(DSRAttributeNames::IDT))
                    mod.edge_to_id = std::get<uint64_t>(mod.attributes[DSRAttributeNames::IDT].value());
                if (mod.attributes.count(DSRAttributeNames::TYPE))
                    mod.type = std::get<std::string>(mod.attributes[DSRAttributeNames::TYPE].value());
        }
    }
    
    void parse_keyframe(const std::string &content, DSREvent &mod){
        // Split nodes and edges
        std::string div = std::string(1, DSRSpecialChars::SLOT) + std::string(1, DSRSpecialChars::K_DIV);
        size_t nodes_edges_div = content.find(div);

        if (nodes_edges_div == std::string::npos)
            throw std::runtime_error("Invalid keyframe format: missing nodes/edges divisor");

        // Nodes sub string from 0 to div
        std::string nodes_section = content.substr(0, nodes_edges_div);
        // Edges sub string from div to end
        std::string edges_section = content.substr(nodes_edges_div + 2);

        // Parse nodes
        parse_nodes_section(nodes_section, mod.nodes);
        // Parse edtges
        parse_edges_section(edges_section, mod.edges);
    }

    void parse_nodes_section(const std::string &section, std::vector<DSR::Node> &nodes){
        size_t pos = 0;
        while (pos < section.size()){
            size_t next_hash = section.find(DSRSpecialChars::SLOT, pos);

            // if there are no more divisors, parse to end
            if (next_hash == std::string::npos)
                next_hash = section.size();

            std::string item_str = section.substr(pos, next_hash - pos);

            if (!item_str.empty() && item_str.size() > 0 && item_str[0] != DSRSpecialChars::K_DIV){
                std::map<std::string, DSR::Attribute> item_attrs;
                parse_attributes(item_str, item_attrs);

                if (!item_attrs.empty()){
                    DSR::Node node;

                    if (item_attrs.count(DSRAttributeNames::ID))
                        node.id(std::get<uint64_t>(item_attrs[DSRAttributeNames::ID].value()));
                    if (item_attrs.count(DSRAttributeNames::NODE_NAME))
                        node.name(std::get<std::string>(item_attrs[DSRAttributeNames::NODE_NAME].value()));
                    if (item_attrs.count(DSRAttributeNames::TYPE))
                        node.type(std::get<std::string>(item_attrs[DSRAttributeNames::TYPE].value()));
                    
                    for (const auto &[name, attr] : item_attrs)
                        node.attrs()[name] = attr;

                    nodes.push_back(node);
                }
            }

            pos = next_hash + 1;
            if (next_hash >= section.size()) break;
        }
    }

    void parse_edges_section(const std::string &section, std::vector<DSR::Edge> &edges){
        size_t pos = 0;
        while (pos < section.size()){
            size_t next_hash = section.find(DSRSpecialChars::SLOT, pos);

            // if there are no more divisors, parse to end
            if (next_hash == std::string::npos)
                next_hash = section.size();

            std::string item_str = section.substr(pos, next_hash - pos);
            
            if (!item_str.empty() && item_str.size() > 0 && item_str[0] != DSRSpecialChars::K_DIV){
                std::map<std::string, DSR::Attribute> item_attrs;
                parse_attributes(item_str, item_attrs);

                if (!item_attrs.empty()){
                    DSR::Edge edge;

                    if (item_attrs.count(DSRAttributeNames::TYPE))
                        edge.type(std::get<std::string>(item_attrs[DSRAttributeNames::TYPE].value()));
                    if (item_attrs.count(DSRAttributeNames::IDF))
                        edge.from(std::get<uint64_t>(item_attrs[DSRAttributeNames::IDF].value()));                        
                    if (item_attrs.count(DSRAttributeNames::IDT))
                        edge.to(std::get<uint64_t>(item_attrs[DSRAttributeNames::IDT].value()));

                    for (const auto &[name, attr] : item_attrs)
                        edge.attrs()[name] = attr;

                    edges.push_back(edge);
                }
            }

            pos = next_hash + 1;
            if (next_hash >= section.size()) break;
        }
    }

    void parse_attributes(const std::string &attrs_str, std::map<std::string, DSR::Attribute> &attrs){
        size_t pos = 0;

        while (pos < attrs_str.size()){
            while (pos < attrs_str.size() && attrs_str[pos] == DSRSpecialChars::ATT_VAL)
                pos++;
            
            if (pos >= attrs_str.size()) break;

            size_t name_div = attrs_str.find(DSRSpecialChars::ATT_NAME, pos); 
            if (name_div == std::string::npos || name_div == pos) break;
            
            size_t type_div = attrs_str.find(DSRSpecialChars::ATT_TYPE, name_div); 
            if (type_div == std::string::npos || type_div == pos) break;
            
            size_t value_div = attrs_str.find(DSRSpecialChars::ATT_VAL, type_div); 

            std::string name = attrs_str.substr(pos, name_div - pos);
            std::string type_code = attrs_str.substr(name_div + 1, type_div - name_div - 1);

            std::string value;
            if (value_div != std::string::npos){
                value = attrs_str.substr(type_div + 1, value_div - type_div - 1);
                pos = value_div + 1;
            } else{
                // last attribute
                size_t end = attrs_str.find(DSRSpecialChars::SLOT, type_div);
                if (end == std::string::npos){
                    value = attrs_str.substr(type_div + 1);
                    pos = attrs_str.size();
                } else{
                    value = attrs_str.substr(type_div + 1, end - type_div - 1);
                    pos = end;
                }
            }

            // Parse attribute as DSR::Attribute if not empty
            if (!name.empty() && !type_code.empty()){
                DSR::Attribute attr;
                attr.value(parse_value_by_code(value, type_code));
                attrs[name] = attr;
            }
        }
    }

    DSR::ValType parse_value_by_code(const std::string &value_str, const std::string &type_code){
        if (type_code == DSRTypeTrait<std::string>::code)
            return value_str;
        if (type_code == DSRTypeTrait<int32_t>::code)
            return static_cast<int32_t>(std::stoi(value_str));
        if (type_code == DSRTypeTrait<float>::code)
            return std::stof(value_str);
        if (type_code == DSRTypeTrait<uint64_t>::code)
            return static_cast<uint64_t>(std::stoull(value_str));
        if (type_code == DSRTypeTrait<double>::code)
            return std::stod(value_str);
        if (type_code == DSRTypeTrait<bool>::code)
            return (value_str == "1" || value_str == "true");
        if (type_code == DSRTypeTrait<uint32_t>::code)
            return static_cast<uint32_t>(std::stoul(value_str));
        if (type_code == DSRTypeTrait<std::vector<float>>::code)
            return parse_vector<float>(value_str);
        if (type_code == DSRTypeTrait<std::vector<uint64_t>>::code)
            return parse_vector<uint64_t>(value_str);
        if (type_code == DSRTypeTrait<std::vector<uint8_t>>::code)
            return parse_vector<uint8_t>(value_str);
        if (type_code == DSRTypeTrait<std::array<float, 2>>::code)
            return parse_array<float, 2>(value_str);
        if (type_code == DSRTypeTrait<std::array<float, 3>>::code)
            return parse_array<float, 3>(value_str);
        if (type_code == DSRTypeTrait<std::array<float, 4>>::code)
            return parse_array<float, 4>(value_str);
        if (type_code == DSRTypeTrait<std::array<float, 6>>::code)
            return parse_array<float, 6>(value_str);
        
        throw std::runtime_error("Unknown type code: " + type_code);
    }

    // Parse vectors
    template<typename T>
    std::vector<T> parse_vector(const std::string &str){
        std::vector<T> result;
        size_t start = str.find('[');
        size_t end = str.find(']');

        if (start == std::string::npos || end == std::string::npos)
            return result;

        std::string content = str.substr(start + 1, end - start - 1);
        std::stringstream ss(content);
        std::string item;

        while (std::getline(ss, item, ',')) {
            if (item.empty()) continue;

            if constexpr (std::is_same_v<T, float>)
                result.push_back(std::stof(item));
            else if constexpr (std::is_same_v<T, uint64_t>)
                result.push_back(std::stoull(item));
            else if constexpr (std::is_same_v<T, uint8_t>)
                result.push_back(static_cast<uint8_t>(std::stoi(item)));
        }

        return result;
    }

    // Parse arrays
    template<typename T, size_t N>
    std::array<T, N> parse_array(const std::string &str){
        auto vec = parse_vector<T>(str);
        std::array<T, N> result{};
        std::copy_n(vec.begin(), std::min(vec.size(), N), result.begin());
        return result;
    }

};


#endif
