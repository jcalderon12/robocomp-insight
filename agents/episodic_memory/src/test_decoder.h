#include "DSRDecoder.h"
#include <iostream>

// Función auxiliar para imprimir un AttributeType
void print_attribute(const std::string& name, const DSRAttributeTypes::AttributeType& attr) {
    std::visit([&name](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        std::cout << "  " << name << " = ";
        
        if constexpr (std::is_same_v<T, std::string>)
            std::cout << value;
        else if constexpr (std::is_arithmetic_v<T>)
            std::cout << value;
        else if constexpr (std::is_same_v<T, std::vector<float>> || 
                           std::is_same_v<T, std::vector<uint64_t>> ||
                           std::is_same_v<T, std::vector<uint8_t>>) {
            std::cout << "[";
            for (size_t i = 0; i < value.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << value[i];
            }
            std::cout << "]";
        } else {
            std::cout << "[array]";
        }
        
        std::cout << std::endl;
    }, attr);
}

void test_decoder() {
    DSRDecoder decoder;
    
    // Ejemplo 1: Modify Node (MN)
    std::string mn_msg = "1771495934978776953#MN#id$ui64:271852530252644352%type$s:object%name$s:test_node%#";
    auto mn_mod = decoder.decode(mn_msg);
    
    std::cout << "=== MODIFY NODE (MN) ===" << std::endl;
    std::cout << "Timestamp: " << mn_mod->timestamp << std::endl;
    std::cout << "Operation: " << mn_mod->modification_type << std::endl;
    std::cout << "Node ID: " << *mn_mod->node_id << std::endl;
    std::cout << "Node Type: " << *mn_mod->type << std::endl;
    std::cout << "Node Name: " << *mn_mod->node_name << std::endl;
    std::cout << std::endl;
    
    // Ejemplo 2: Modify Node Attrs (MNA)
    std::string mna_msg = "1771495936052216633#MNA#id$ui64:271852530252644352%pos_x$f:21,000000%pos_y$f:353,000000%#";
    auto mna_mod = decoder.decode(mna_msg);
    
    std::cout << "=== MODIFY NODE ATTRS (MNA) ===" << std::endl;
    std::cout << "Timestamp: " << mna_mod->timestamp << std::endl;
    std::cout << "Operation: " << mna_mod->modification_type << std::endl;
    std::cout << "Node ID: " << *mna_mod->node_id << std::endl;
    std::cout << "Modified attributes:" << std::endl;
    for (const auto& [name, value] : mna_mod->attributes) {
        if (name != DSRAttributeNames::ID) {  // Saltar ID estructural
            print_attribute(name, value.value());
        }
    }
    std::cout << std::endl;
    
    // Ejemplo 3: Modify Edge (ME)
    std::string me_msg = "1771495936478005930#ME#idf$ui64:200%idt$ui64:271852530252644352%type$s:has%#";
    auto me_mod = decoder.decode(me_msg);
    
    std::cout << "=== MODIFY EDGE (ME) ===" << std::endl;
    std::cout << "Timestamp: " << me_mod->timestamp << std::endl;
    std::cout << "Operation: " << me_mod->modification_type << std::endl;
    std::cout << "Edge From: " << *me_mod->edge_from_id << std::endl;
    std::cout << "Edge To: " << *me_mod->edge_to_id << std::endl;
    std::cout << "Type: " << *me_mod->type << std::endl;
    std::cout << std::endl;
    
    // Ejemplo 4: Modify Edge Attrs (MEA)
    std::string mea_msg = "1771495679837117126#MEA#idf$ui64:200%idt$ui64:271852530252644352%type$s:has%robot_target_x$f:61,000000%#";
    auto mea_mod = decoder.decode(mea_msg);
    
    std::cout << "=== MODIFY EDGE ATTRS (MEA) ===" << std::endl;
    std::cout << "Timestamp: " << mea_mod->timestamp << std::endl;
    std::cout << "Operation: " << mea_mod->modification_type << std::endl;
    std::cout << "Edge: " << *mea_mod->edge_from_id << " -> " << *mea_mod->edge_to_id << std::endl;
    std::cout << "Edge Type: " << *mea_mod->edge_type << std::endl;
    std::cout << "Modified attributes:" << std::endl;
    for (const auto& [name, value] : mea_mod->attributes) {
        // Saltar atributos estructurales
        if (name != DSRAttributeNames::IDF && 
            name != DSRAttributeNames::IDT && 
            name != DSRAttributeNames::TYPE) {
            print_attribute(name, valuevalue());
        }
    }
    std::cout << std::endl;
    
    // Ejemplo 5: Delete Edge (DE)
    std::string de_msg = "1771495681355865997#DE#idf$ui64:200%idt$ui64:271852530252644352%type$s:RT%#";
    auto de_mod = decoder.decode(de_msg);
    
    std::cout << "=== DELETE EDGE (DE) ===" << std::endl;
    std::cout << "Timestamp: " << de_mod->timestamp << std::endl;
    std::cout << "Operation: " << de_mod->modification_type << std::endl;
    std::cout << "Deleting edge: " << *de_mod->edge_from << " -> " << *de_mod->edge_to << std::endl;
    std::cout << "Edge Type: " << *de_mod->edge_type << std::endl;
    std::cout << std::endl;
    
    // Ejemplo 6: Delete Node (DN)
    std::string dn_msg = "1771495682204704065#DN#id$ui64:271852530252644352%#";
    auto dn_mod = decoder.decode(dn_msg);
    
    std::cout << "=== DELETE NODE (DN) ===" << std::endl;
    std::cout << "Timestamp: " << dn_mod->timestamp << std::endl;
    std::cout << "Operation: " << dn_mod->modification_type << std::endl;
    std::cout << "Deleting node ID: " << *dn_mod->node_id << std::endl;
    std::cout << std::endl;
    
    // Ejemplo 7: Keyframe (K) - simplificado
    std::string k_msg = "1771495929540436347#K#name$s:robot%type$s:robot%id$ui64:200%#name$s:root%type$s:root%id$ui64:100%#@type$s:RT%idf$ui64:100%idt$ui64:200%#type$s:has%idf$ui64:100%idt$ui64:200%#@";
    auto k_mod = decoder.decode(k_msg);
    
    std::cout << "=== KEYFRAME (K) ===" << std::endl;
    std::cout << "Timestamp: " << k_mod->timestamp << std::endl;
    std::cout << "Operation: " << k_mod->modification_type << std::endl;
    std::cout << "Number of nodes: " << k_mod->nodes.size() << std::endl;
    std::cout << "Number of edges: " << k_mod->edges.size() << std::endl;
    
    std::cout << "\nNodes:" << std::endl;
    for (size_t i = 0; i < k_mod->nodes.size(); ++i) {
        std::cout << "  Node " << i << ":" << std::endl;
        for (const auto& [name, value] : k_mod->nodes[i].attrs()) {
            print_attribute(name, value);
        }
    }
    
    std::cout << "\nEdges:" << std::endl;
    for (size_t i = 0; i < k_mod->edges.size(); ++i) {
        std::cout << "  Edge " << i << ":" << std::endl;
        for (const auto& [name, value] : k_mod->edges[i].attrs()) {
            print_attribute(name, value);
        }
    }    
}
