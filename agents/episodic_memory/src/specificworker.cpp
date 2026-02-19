/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif
		
		qInstallMessageHandler([](QtMsgType, const QMessageLogContext&, const QString&) {});

		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}

		historic_graph = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "root_dsr.json", true, 1);
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	//G->write_to_json_file("./"+agent_name+".json");
	delete historic_window;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;
	//dsr update signals
	connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
	connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
	connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_node_attrs_slot);
	connect(G.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &SpecificWorker::modify_edge_attrs_slot);
	connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
	connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

	/***
	Custom Widget
	In addition to the predefined viewers, Graph Viewer allows you to add various widgets designed by the developer.
	The add_custom_widget_to_dock method is used. This widget can be defined like any other Qt widget,
	either with a QtDesigner or directly from scratch in a class of its own.
	The add_custom_widget_to_dock method receives a name for the widget and a reference to the class instance.
	***/

	graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
	//graph_viewer->add_custom_widget_to_dock("CustomWidget", &custom_widget);

	historic_window = new QMainWindow;
	historic_viewer = std::make_unique<DSR::DSRViewer>(historic_window, historic_graph, current_opts, main);
	historic_window->show();

	historic_debugger_ui.setupUi(&historic_debugger_widget);
	connect(historic_debugger_ui.local_changes_scroll_bar, &QScrollBar::valueChanged, this, &SpecificWorker::local_changes_management);
	connect(historic_debugger_ui.global_changes_scroll_bar, &QScrollBar::valueChanged, this, &SpecificWorker::global_changes_management);
	historic_viewer->add_custom_widget_to_dock("Historic debugger", &historic_debugger_widget);

	keyframe_period = std::chrono::milliseconds(configLoader.get<int>("KeyframePeriod"));
	time_check = std::chrono::system_clock::now(); 
	generate_keyframe();

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

	test_decoder();
}



void SpecificWorker::compute()
{
	// Generate a new keyframe every X seconds
	// auto time_now = std::chrono::system_clock::now();
	// if (time_now - time_check >= keyframe_period){
	// 	time_check = std::chrono::system_clock::now();		
	// 	generate_keyframe();
	// }
}



void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}



//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}


void SpecificWorker::local_changes_management(int value){
	// historic_debugger_ui.slider_label->setNum(value);
	std::cout << "Local changed value: " << value << std::endl;
}


void SpecificWorker::global_changes_management(int value){
	// historic_debugger_ui.slider_label->setNum(value);
	std::cout << "Global changed value: " << value << std::endl;
}


void SpecificWorker::modify_node_slot(std::uint64_t id, const std::string &type){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag) {
		std::cout << "Modify node slot - id: " << id << " type: " << type << " ";
		std::cout << "new_timestamp: " << new_timestamp << std::endl;
	}
	else { 
		auto dsr_data_optional = assemble_string(new_timestamp, DSRSpecialChars::MN, id, type, {});
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
		changes_map[new_timestamp] = dsr_data;
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;
	}			
}


void SpecificWorker::modify_node_attrs_slot(std::uint64_t id, const std::vector<std::string>& att_names){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag) {
		std::cout << "Modify node attrs slot - id: " << id << " att_names_size: " << att_names.size() << " att_names: ";
		for (const auto &att : att_names){ 
			std::cout << att << " "; 
			auto node_optional = G->get_node(id);
			if (node_optional.has_value())
			{	
				auto node = node_optional.value();
				auto timestamp_optional = G->get_attrib_timestamp_by_name(node, att);
				if (timestamp_optional.has_value()){
					auto timestamp = timestamp_optional.value();
					std::cout << " time: " << timestamp << " ";
				}
			}
		}
		std::cout << std::endl;
	}
	else {
		auto dsr_data_optional = assemble_string(new_timestamp, DSRSpecialChars::MNA, id, "", att_names); // no type/tag
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
		changes_map[new_timestamp] = dsr_data;
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;
	}
}

void SpecificWorker::modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag){
		auto edges_optional = G->get_edges(from);
		if (edges_optional.has_value()){
			auto edge_map = edges_optional.value();
			for (const auto &e : edge_map){
				auto key = e.first;
				std::cout << key.second << std::endl;
			}

			// auto keys = edges_optional.value().keys();
			// std::cout << keys.size() << std::endl;
		}
		std::cout << "Modify edge slot - from_id: " << from << " to_id: " << to << " type: " << type << std::endl;
	}
	else
	{
		auto dsr_data_optional = assemble_string(new_timestamp, DSRSpecialChars::ME, std::make_tuple(from, to), type, {});
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
		changes_map[new_timestamp] = dsr_data;
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;
	}
}

void SpecificWorker::modify_edge_attrs_slot(std::uint64_t from, std::uint64_t to, const std::string &type, const std::vector<std::string>& att_names){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag){
		std::cout << "Modify edge attrs slot - from_id: " << from << " to_id: " << to << " type: " << type; 
		std::cout << "\n att_names_size: " << att_names.size() << " att_names: ";
		for (const auto &att : att_names) std::cout << att << " ";
		std::cout << std::endl;
	}
	else{
		auto dsr_data_optional = assemble_string(new_timestamp, DSRSpecialChars::MEA, std::make_tuple(from, to), type, att_names);
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
		changes_map[new_timestamp] = dsr_data;
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;		
	}
}

void SpecificWorker::del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag)
		std::cout << "Delete edge slot" << std::endl;
	else{
		auto dsr_data_optional = assemble_string(new_timestamp, DSRSpecialChars::DE, std::make_tuple(from, to), edge_tag, {});
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
		changes_map[new_timestamp] = dsr_data;
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;
	}
}

void SpecificWorker::del_node_slot(std::uint64_t from){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag)
		std::cout << "Delete node slot" << std::endl;
	else{
		auto dsr_data_optional = assemble_string(new_timestamp, DSRSpecialChars::DN, from, "", {});
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
		changes_map[new_timestamp] = dsr_data;
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;
	}
}


std::tuple<std::string, std::string> SpecificWorker::attribute_value_and_type_to_string(const auto &att) {
    return std::visit([](const auto& value) -> std::tuple<std::string, std::string> {
        using T = std::decay_t<decltype(value)>;
        
        if constexpr (std::is_same_v<T, std::string>)
			return {value, DSRTypeTrait<T>::code};
		else if constexpr (std::is_arithmetic_v<T>)
			return {std::to_string(value), DSRTypeTrait<T>::code};
		else {
			std::string out = "[";
			bool first = true;
			for (const auto &x : value) {
				if (!first) out += ",";
				out += std::to_string(x);
				first = false;
			}
			out += "]";
			return {out, DSRTypeTrait<T>::code};
		}
    }, att.value());
}


std::optional<std::string> SpecificWorker::assemble_string(const auto &timestamp, const std::string &slot, const std::variant<std::uint64_t, std::tuple<uint64_t, uint64_t>> &variant_id,
			const std::string &type, const std::vector<std::string> &att_names)
{
	std::optional<DSR::Node> node_optional;
	std::optional<DSR::Edge> edge_optional;
	DSR::Node node;
	DSR::Edge edge;

	// timestamp + slot 
	std::string dsr_data = "";
	dsr_data += std::to_string(timestamp); dsr_data += DSRSpecialChars::SLOT; 
	dsr_data += slot; dsr_data += DSRSpecialChars::SLOT;

	// Node ID
	if(std::holds_alternative<uint64_t>(variant_id)){ 
		auto node_id = std::get<uint64_t>(variant_id);
		node_optional = G->get_node(node_id);
		if(node_optional.has_value())
			node = node_optional.value();

		// ID
		dsr_data += DSRAttributeNames::ID; dsr_data += DSRSpecialChars::ATT_NAME;
		dsr_data += DSRTypeTrait<uint64_t>::code; dsr_data += DSRSpecialChars::ATT_TYPE;
		dsr_data += std::to_string(node_id); dsr_data += DSRSpecialChars::ATT_VAL;	
	}
	// Edge ID
	else if (std::holds_alternative<std::tuple<uint64_t, uint64_t>>(variant_id)){ 
		auto [from_id, to_id] = std::get<std::tuple<uint64_t, uint64_t>>(variant_id);
		if(slot != DSRSpecialChars::DE)
			edge_optional = G->get_edge(from_id, to_id, type);
		if(edge_optional.has_value())
			edge = edge_optional.value();
		
		// from ID
		dsr_data += DSRAttributeNames::IDF; dsr_data += DSRSpecialChars::ATT_NAME;
		dsr_data += DSRTypeTrait<uint64_t>::code; dsr_data += DSRSpecialChars::ATT_TYPE;
		dsr_data += std::to_string(from_id); dsr_data += DSRSpecialChars::ATT_VAL;
		
		// to ID
		dsr_data += DSRAttributeNames::IDT; dsr_data += DSRSpecialChars::ATT_NAME;
		dsr_data += DSRTypeTrait<uint64_t>::code; dsr_data += DSRSpecialChars::ATT_TYPE;
		dsr_data += std::to_string(to_id); dsr_data += DSRSpecialChars::ATT_VAL;
		
	}
	
	// Node / Edge: type - Edge: tag (when deleted) 
	if (!type.empty()) {
		dsr_data += DSRAttributeNames::TYPE; dsr_data += DSRSpecialChars::ATT_NAME; 
		dsr_data += DSRTypeTrait<std::string>::code; dsr_data += DSRSpecialChars::ATT_TYPE;
		dsr_data += type; dsr_data += DSRSpecialChars::ATT_VAL;
	}
	
	// Node: name
	if (slot == DSRSpecialChars::MN && node_optional.has_value()) {
		dsr_data += DSRAttributeNames::NODE_NAME; dsr_data += DSRSpecialChars::ATT_NAME; 
		dsr_data += DSRTypeTrait<std::string>::code; dsr_data += DSRSpecialChars::ATT_TYPE;
		dsr_data += node.name(); dsr_data += DSRSpecialChars::ATT_VAL;
	} 

	// Attributes
	if(!att_names.empty()){
		for (const auto &att_name : att_names){
			if (att_name.empty()) continue;
			std::optional<DSR::Attribute> att_optional;
			if (std::holds_alternative<std::uint64_t>(variant_id)){ // Node
				if (node.attrs().count(att_name) > 0)
					att_optional = node.attrs()[att_name];}
			else if (std::holds_alternative<std::tuple<uint64_t, uint64_t>>(variant_id)){  // Edge				
				if (edge.attrs().count(att_name) > 0)
					att_optional = edge.attrs()[att_name];}

			if (att_optional.has_value()){
				auto [att_value, att_type] = attribute_value_and_type_to_string(att_optional.value());
				if (!att_value.empty()){
					dsr_data += att_name; dsr_data += DSRSpecialChars::ATT_NAME;
					dsr_data += att_type; dsr_data += DSRSpecialChars::ATT_TYPE;
					dsr_data += att_value; dsr_data += DSRSpecialChars::ATT_VAL;
				}
			}
		}
	}
	dsr_data += DSRSpecialChars::SLOT;
	return std::make_optional(std::move(dsr_data));
}


void SpecificWorker::generate_keyframe(){
	std::vector<std::string> nodes_str_v;
	std::vector<std::string> edges_str_v;
	std::string dsr_kdata = "";

	// timestamp + K
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_check.time_since_epoch()).count(); 
	dsr_kdata += std::to_string(new_timestamp); dsr_kdata += DSRSpecialChars::SLOT;
	dsr_kdata += DSRSpecialChars::K; dsr_kdata += DSRSpecialChars::SLOT;

	auto nodes = G->get_nodes();
	for (const auto &node : nodes){
		// node info
		std::string node_data = "";
		
		// node name
		node_data += DSRAttributeNames::NODE_NAME; node_data += DSRSpecialChars::ATT_NAME;
		node_data += DSRTypeTrait<std::string>::code; node_data += DSRSpecialChars::ATT_TYPE;
		node_data += node.type(); node_data += DSRSpecialChars::ATT_VAL;
		
		// node type
		node_data += DSRAttributeNames::TYPE; node_data += DSRSpecialChars::ATT_NAME;
		node_data += DSRTypeTrait<std::string>::code; node_data += DSRSpecialChars::ATT_TYPE;
		node_data += node.name(); node_data += DSRSpecialChars::ATT_VAL;
		
		// node id
		node_data += DSRAttributeNames::ID; node_data += DSRSpecialChars::ATT_NAME;
		node_data += DSRTypeTrait<uint64_t>::code; node_data += DSRSpecialChars::ATT_TYPE;
		const auto id_optional = G->get_id_from_name(node.name());
		if (id_optional.has_value()) {
			node_data += std::to_string(id_optional.value()); node_data += DSRSpecialChars::ATT_VAL; }
		
		// node attrs
		auto attrs = node.attrs();
		for (const auto &att : attrs){
			auto att_val_type = attribute_value_and_type_to_string(att.second);
			node_data += att.first; node_data += DSRSpecialChars::ATT_NAME;
			node_data += std::get<1>(att_val_type); node_data += DSRSpecialChars::ATT_TYPE;
			node_data += std::get<0>(att_val_type); node_data += DSRSpecialChars::ATT_VAL;			
		}
		node_data += DSRSpecialChars::SLOT; 
		nodes_str_v.push_back(node_data);
				
		// edge info
		std::string edge_data = "";
		auto node_edges_optional = G->get_edges(node.id());
		if (node_edges_optional.has_value()){
			auto edges_map = node_edges_optional.value();
			for (const auto &edge_map_element : edges_map){
				auto key_idto_type = edge_map_element.first; 
				auto edge = edge_map_element.second;

				// edge type
				edge_data += DSRAttributeNames::TYPE; edge_data += DSRSpecialChars::ATT_NAME; 
				edge_data += DSRTypeTrait<std::string>::code; edge_data += DSRSpecialChars::ATT_TYPE;
				edge_data += key_idto_type.second; edge_data += DSRSpecialChars::ATT_VAL;
				
				// edge id from
				edge_data += DSRAttributeNames::IDF; edge_data += DSRSpecialChars::ATT_NAME; 
				edge_data += DSRTypeTrait<uint64_t>::code; edge_data += DSRSpecialChars::ATT_TYPE;
				if (id_optional.has_value()) {
					edge_data += std::to_string(id_optional.value()); edge_data += DSRSpecialChars::ATT_VAL; }

				// edge id to
				edge_data += DSRAttributeNames::IDT; edge_data += DSRSpecialChars::ATT_NAME; 
				edge_data += DSRTypeTrait<uint64_t>::code; edge_data += DSRSpecialChars::ATT_TYPE;
				edge_data += std::to_string(key_idto_type.first); edge_data += DSRSpecialChars::ATT_VAL;

				// edge attrs
				auto attrs = edge.attrs();
				for (const auto &att : attrs){
					auto att_val_type = attribute_value_and_type_to_string(att.second);
					edge_data += att.first; edge_data += DSRSpecialChars::ATT_NAME;
					edge_data += std::get<1>(att_val_type); edge_data += DSRSpecialChars::ATT_TYPE;
					edge_data += std::get<0>(att_val_type); edge_data += DSRSpecialChars::ATT_VAL;
				}

				edge_data += DSRSpecialChars::SLOT;
			}
			edges_str_v.push_back(edge_data);
		} 
	}

	for (const auto &node_str : nodes_str_v) 
		dsr_kdata += node_str;
	dsr_kdata += DSRSpecialChars::K_DIV;
	for (const auto &edge_str : edges_str_v) 
		dsr_kdata += edge_str;
	dsr_kdata += DSRSpecialChars::K_DIV;

	changes_map[new_timestamp] = dsr_kdata;
	std::cout << dsr_kdata << std::endl;
	std::cout << std::endl;
}
