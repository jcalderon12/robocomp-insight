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
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	//G->write_to_json_file("./"+agent_name+".json");
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

    //initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

}



void SpecificWorker::compute()
{

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
		auto optional_node = G->get_node(id);
		if (!optional_node.has_value()){
			std::cout << __FUNCTION__ << " - optional_node has no value"; return; }

		auto node = optional_node.value();
		std::cout << __FUNCTION__ << " - " << new_timestamp << "#MN%" << id << "%" << type << "%" << node.name() << std::endl;
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
		std::string dsr_data = "";
		dsr_data += std::to_string(new_timestamp); dsr_data += "#MNA%"; dsr_data += std::to_string(id);
		auto node_optional = G->get_node(id);
		if(!node_optional.has_value()){std::cerr << __FUNCTION__ << "node_optional has no value" << std::endl; return;}
		auto node = node_optional.value();
		for (const auto &att_name : att_names){
			if (att_name.size() > 0){
				auto att = node.attrs()[att_name];
				std::string att_value_str = attribute_value_to_string(att);
				dsr_data += "%"; dsr_data += att_value_str;
			}
		}			
		std::cout << __FUNCTION__ << " - " << dsr_data << std::endl;
	}
}

void SpecificWorker::modify_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag)
		std::cout << "Modify edge slot - from_id: " << from << " to_id: " << to << " type: " << type << std::endl;
	else
		std::cout << __FUNCTION__ << " - " << new_timestamp << "#ME%" << from << "%" << to << "%" << type << std::endl;
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
		std::cout << __FUNCTION__ << " - " << new_timestamp << "#MEA%" << from << "%" << to << "%" << type;
		for (const auto &att : att_names)
			std::cout << "%" << att;
		std::cout << std::endl;
	}
}

void SpecificWorker::del_edge_slot(std::uint64_t from, std::uint64_t to, const std::string &edge_tag){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag)
		std::cout << "Delete edge slot" << std::endl;
	else
		std::cout << __FUNCTION__ << " - " << new_timestamp << "#DE%" << from << "%" << to << "%" << edge_tag << std::endl;
}

void SpecificWorker::del_node_slot(std::uint64_t from){
	const auto time_now = std::chrono::system_clock::now();
	auto new_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now.time_since_epoch()).count();
	
	if (!string_check_flag)
		std::cout << "Delete node slot" << std::endl;
	else
		std::cout << __FUNCTION__ << " - " << new_timestamp << "#DN%" << from << std::endl;
}


std::string SpecificWorker::value_to_string(const AttributeType &value){
	return std::visit([](const auto &value) -> std::string {
		using T = std::decay_t<decltype(value)>;
		if constexpr (std::is_same_v<T, std::string>)
			return value;
		else if constexpr (std::is_arithmetic_v<T>)
			return std::to_string(value);
		else {
			std::string out = "[";
			bool first = true;
			for (const auto &x : value) {
				if (!first) out += ", ";
				out += std::to_string(x);
				first = false;
			}
			out += "]";
			return out;
		}
	}, value);
}


std::string SpecificWorker::attribute_value_to_string(const auto &att) {
    std::string att_value_str;

    switch (att.value().index()) {
        case DSR::Types::STRING:
            att_value_str += value_to_string(std::get<std::string>(att.value()));
            break;
        case DSR::Types::INT:
            att_value_str += value_to_string(std::get<int32_t>(att.value()));
            break;
        case DSR::Types::FLOAT:
            att_value_str += value_to_string(std::get<float>(att.value()));
            break;
        case DSR::Types::FLOAT_VEC:
            att_value_str += value_to_string(std::get<std::vector<float>>(att.value()));
            break;
        case DSR::Types::BOOL:
            att_value_str += value_to_string(std::get<bool>(att.value()));
            break;
        case DSR::Types::BYTE_VEC:
            att_value_str += value_to_string(std::get<std::vector<uint8_t>>(att.value()));
            break;
        case DSR::Types::UINT:
            att_value_str += value_to_string(std::get<uint32_t>(att.value()));
            break;
        case DSR::Types::UINT64:
            att_value_str += value_to_string(std::get<uint64_t>(att.value()));
            break;
        case DSR::Types::DOUBLE:
            att_value_str += value_to_string(std::get<double>(att.value()));
            break;
        case DSR::Types::U64_VEC:
            att_value_str += value_to_string(std::get<std::vector<uint64_t>>(att.value()));
            break;
        case DSR::Types::VEC2:
            att_value_str += value_to_string(std::get<std::array<float, 2>>(att.value()));
            break;
        case DSR::Types::VEC3:
            att_value_str += value_to_string(std::get<std::array<float, 3>>(att.value()));
            break;
        case DSR::Types::VEC4:
            att_value_str += value_to_string(std::get<std::array<float, 4>>(att.value()));
            break;
        case DSR::Types::VEC6:
            att_value_str += value_to_string(std::get<std::array<float, 6>>(att.value()));
            break;
    }

    return att_value_str;
}

