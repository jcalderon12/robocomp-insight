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

    G2 = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, "episodic_dsr.json", true, 1); // Init nodes

	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	//G->write_to_json_file("./"+agent_name+".json");
	delete window2;

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
	
	window2 = new QMainWindow;
	graph_viewer2 = std::make_unique<DSR::DSRViewer>(window2, G2, DSR::DSRViewer::view::graph,  DSR::DSRViewer::view::none);
	window2->show();

	//graph_viewer->add_custom_widget_to_dock("CustomWidget", &custom_widget);

    //initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 
	
	// - DECODE TEST
	// std::string data = "pos_x$i:289%pos_y$f:38,000555200#possad$i:285559%pos_asdsady$f:35558,000555200#";
    // std::cout << "Procesando string: " << data << "\n\n";

    // auto it = data.begin();
    // auto end = data.end();
	
	// std::cout<< "ssssssssssssssssss"<<std::endl;
    // decode_string(it, end);
	// std::cout<< "ssssssssssssssssss"<<std::endl;
    // decode_string(it, end);
	// std::cout<< "ssssssssssssssssss"<<std::endl;
    // decode_string(it, end);
	// std::cout<< "ssssssssssssssssss"<<std::endl;
	// - DECODE TEST

}



void SpecificWorker::compute()
{
	auto node_optional = G->get_node("root");
	if (node_optional.has_value())
	{
		std::cout<< node_optional.value().id()<< std::endl;
	}

	node_optional = G2->get_node("root");
	if (node_optional.has_value())
	{
		std::cout<< node_optional.value().id()<< std::endl;
	}

	std::cout<<"------------------------------------------------"<< std::endl;

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

void SpecificWorker::modify_node_slot(std::uint64_t id, const std::string &type){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag) {
		std::cout << "Modify node slot - id: " << id << " type: " << type << " ";
		std::cout << "new_timestamp: " << new_timestamp << std::endl;
	}
	else { 
		auto dsr_data_optional = assemble_string(new_timestamp, SChars.MN, id, type, {});
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
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
		auto dsr_data_optional = assemble_string(new_timestamp, SChars.MNA, id, "", att_names); // no type/tag
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;
	}
}

void SpecificWorker::modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag)
		std::cout << "Modify edge slot - from_id: " << from << " to_id: " << to << " type: " << type << std::endl;
	else
	{
		auto dsr_data_optional = assemble_string(new_timestamp, SChars.ME, std::make_tuple(from, to), type, {});
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
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
		auto dsr_data_optional = assemble_string(new_timestamp, SChars.MEA, std::make_tuple(from, to), type, att_names);
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;		
	}
}

void SpecificWorker::del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag)
		std::cout << "Delete edge slot" << std::endl;
	else{
		auto dsr_data_optional = assemble_string(new_timestamp, SChars.DE, std::make_tuple(from, to), edge_tag, {});
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;
	}
}

void SpecificWorker::del_node_slot(std::uint64_t from){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag)
		std::cout << "Delete node slot" << std::endl;
	else{
		auto dsr_data_optional = assemble_string(new_timestamp, SChars.DN, from, "", {});
		if (!dsr_data_optional.has_value()) {std::cerr << __FUNCTION__ << " - dsr_data_optional has no value" << std::endl; return;}
		auto dsr_data = dsr_data_optional.value();
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


std::optional<std::string> SpecificWorker::assemble_string(const auto &timestamp, const std::string &slot, const std::variant<std::uint64_t, std::tuple<uint64_t, uint64_t>> &id_variant,
			const std::string &type, const std::vector<std::string> &att_names)
{
	constexpr uint64_t NODE_ONLY = -1;
	uint64_t id_node, id_from, id_to;
	std::optional<DSR::Node> node_optional;
	std::optional<DSR::Edge> edge_optional;
	DSR::Node node;
	DSR::Edge edge;

	// timestamp + slot 
	std::string dsr_data = "";
	dsr_data += std::to_string(timestamp); dsr_data += SChars.SLOT; 
	dsr_data += slot; dsr_data += SChars.SLOT;

	// id: node / edge
	auto id_tuple = std::visit([NODE_ONLY](const auto &value) -> std::tuple<uint64_t, uint64_t> {
		using T = std::decay_t<decltype(value)>;
		if constexpr (std::is_same_v<T, std::uint64_t>)
			return {value, NODE_ONLY};
		else
			return {std::get<0>(value), std::get<1>(value)};
	}, id_variant);
	
	if(std::get<1>(id_tuple) == NODE_ONLY){ // Node
		id_node = std::get<0>(id_tuple);
		dsr_data += "id"; dsr_data += SChars.ATT_NAME;
		dsr_data += std::to_string(id_node); dsr_data += SChars.ATT_VAL;	
		
		node_optional = G->get_node(id_node);
		if(node_optional.has_value())
			node = node_optional.value();
	}
	else{ // Edge
		id_from = std::get<0>(id_tuple);
		id_to = std::get<1>(id_tuple);
		dsr_data += "idf"; dsr_data += SChars.ATT_NAME;
		dsr_data += std::to_string(id_from); dsr_data += SChars.ATT_VAL;
		dsr_data += "idt"; dsr_data += SChars.ATT_NAME;
		dsr_data += std::to_string(id_to); dsr_data += SChars.ATT_VAL;
		
		if(slot != SChars.DE)
			edge_optional = G->get_edge(id_from, id_to, type);
		if(edge_optional.has_value())
			edge = edge_optional.value();
	}
	
	// type: node / edge - also tag: edge (when deleted) 
	if (!type.empty())
		{dsr_data += type; dsr_data += SChars.ATT_VAL;}
	// name: node
	if (slot == SChars.MN && node_optional.has_value())
		{auto node = node_optional.value(); dsr_data += node.name(); dsr_data += SChars.ATT_VAL;} 

	if(!att_names.empty()){
		for (const auto &att_name : att_names){
			if (att_name.size() > 0){
				if(node_optional.has_value()){ // Node
					auto att = node.attrs()[att_name];
					auto att_val_type = attribute_value_and_type_to_string(att);
					dsr_data += att_name; dsr_data += SChars.ATT_NAME; 
					dsr_data += std::get<1>(att_val_type); dsr_data += SChars.ATT_TYPE;
					dsr_data += std::get<0>(att_val_type); dsr_data += SChars.ATT_VAL;
				}
				else if(edge_optional.has_value()) { // Edge
					auto att = edge.attrs()[att_name];
					auto att_val_type = attribute_value_and_type_to_string(att);
					dsr_data += att_name; dsr_data += SChars.ATT_NAME;
					dsr_data += std::get<1>(att_val_type); dsr_data += SChars.ATT_TYPE;
					dsr_data += std::get<0>(att_val_type); dsr_data += SChars.ATT_VAL;
				}
			}
		}
	}
	dsr_data.back() = SChars.SLOT;
	
	return std::make_optional(std::move(dsr_data));
}