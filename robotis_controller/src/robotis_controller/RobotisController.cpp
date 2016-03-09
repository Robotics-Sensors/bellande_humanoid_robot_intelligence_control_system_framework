/*
 * RobotisController.cpp
 *
 *  Created on: 2016. 1. 15.
 *      Author: zerom
 */

#include <ros/package.h>
#include <ros/callback_queue.h>

#include "robotis_controller/RobotisController.h"

#define INDIRECT_ADDRESS "indirect_address_1"

using namespace ROBOTIS;

RobotisController *RobotisController::unique_instance_ = new RobotisController();

// INDIRECT ADDR : 634 -> Present Position (4 Byte)
// INDIRECT ADDR : 638 -> Present Velocity (4 Byte)
// INDIRECT ADDR : 642 -> Present Current (2 Byte)
// INDIRECT ADDR : 644 -> Input Voltage (2 Byte)
// INDIRECT ADDR : 646 -> Temperature (1 byte)
// INDIRECT ADDR : 647 -> External Port Data 1 (2 byte)
// INDIRECT ADDR : 649 -> External Port Data 2 (2 byte)
// INDIRECT ADDR : 651 -> External Port Data 3 (2 byte)
// INDIRECT ADDR : 653 -> External Port Data 4 (2 byte)
const UINT16_T PRESENT_POSITION_ADDR    = 634;
const UINT16_T TORQUE_ENABLE_ADDR       = 562;
const UINT16_T GOAL_POSITION_ADDR       = 596;
const UINT16_T GOAL_VELOCITY_ADDR       = 600;
const UINT16_T GOAL_ACCELERATION_ADDR   = 606;
const UINT16_T EXT_PORT_DATA1_ADDR      = 647;
const UINT16_T EXT_PORT_DATA2_ADDR      = 649;
const UINT16_T EXT_PORT_DATA3_ADDR      = 651;
const UINT16_T EXT_PORT_DATA4_ADDR      = 653;

RobotisController::RobotisController()
: is_timer_running_(false),
  stop_timer_(false),
  timer_thread_(0),
  controller_mode_(MOTION_MODULE_MODE),
  DEBUG_PRINT(false),
  robot(0)
{
	direct_sync_write_.clear();
}

void RobotisController::InitSyncWrite()
{
	ROS_INFO("FIRST BULKREAD");
	// bulkread twice
	for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
		_it->second->TxRxPacket();
	for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
	{
	    int _error_cnt = 0;
	    int _result = COMM_SUCCESS;
        do {
            if(++_error_cnt > 10)
            {
                ROS_ERROR("[RobotisController] bulk read fail!!");
                exit(-1);
            }
            usleep(10*1000);
            _result = _it->second->TxRxPacket();
        } while (_result != COMM_SUCCESS);
	}
	ROS_INFO("FIRST BULKREAD END");

	// clear syncwrite param setting
	for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write.begin(); _it != port_to_sync_write.end(); _it++)
		_it->second->ClearParam();

	// set init syncwrite param(from data of bulkread)
	for(std::map<std::string, Dynamixel*>::iterator _it = robot->dxls.begin(); _it != robot->dxls.end(); _it++)
	{
		INT32_T _pos = 0;

		std::string _joint_name = _it->first;
		Dynamixel  *_dxl        = _it->second;

		bool _res = false;
		_res = port_to_bulk_read[_dxl->port_name]->GetData(_dxl->id, PRESENT_POSITION_ADDR, (UINT32_T*)&_pos);
		if(_res == true)
		{
			_dxl->dxl_state->present_position = _dxl->ConvertValue2Radian(_pos) - _dxl->dxl_state->position_offset; // remove offset;
			_dxl->dxl_state->goal_position = _dxl->dxl_state->present_position;
		}

		UINT8_T _sync_write_data[4];
		_sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos));
		_sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos));
		_sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos));
		_sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos));

		port_to_sync_write[_dxl->port_name]->AddParam(_dxl->id, _sync_write_data);
	}
}

bool RobotisController::Initialize(const std::string robot_file_path, const std::string init_file_path)
{
	std::string _dev_desc_dir_path  = ros::package::getPath("robotis_device") + "/devices";

	// load robot info : port , device
	robot = new Robot(robot_file_path, _dev_desc_dir_path);
	// TODO: fill joint_ctrl_module_


	// TODO: TEMPORARY CODE !!
	/* temporary code start */
	PacketHandler *_protocol2_handler = PacketHandler::GetPacketHandler(2.0);

	for(std::map<std::string, PortHandler *>::iterator _it = robot->ports.begin(); _it != robot->ports.end(); _it++)
	{
		port_to_bulk_read[_it->first] = new GroupBulkRead(_it->second, _protocol2_handler);
		port_to_sync_write[_it->first] = new GroupSyncWrite(_it->second, _protocol2_handler, GOAL_POSITION_ADDR, 4);
	}


    // TODO:
    // for loop ping --> no response : return false
    for(std::map<std::string, Dynamixel*>::iterator _it = robot->dxls.begin(); _it != robot->dxls.end(); _it++)
    {
        std::string _joint_name = _it->first;
        Dynamixel  *_dxl        = _it->second;
        if(Ping(_joint_name) != 0)
        {
            usleep(10*1000);
            if(Ping(_joint_name) != 0)
                ROS_ERROR("JOINT[%s] does NOT response!!", _joint_name.c_str());
        }
    }

    // joint(dynamixel) initialize
	if(DEBUG_PRINT) ROS_WARN("INIT FILE LOAD");

	YAML::Node _doc;
	try{
	    _doc = YAML::LoadFile(init_file_path.c_str());

	    for(YAML::const_iterator _it_doc = _doc.begin(); _it_doc != _doc.end(); _it_doc++)
	    {
	        std::string _joint_name = _it_doc->first.as<std::string>();

	        YAML::Node _joint_node = _doc[_joint_name];
	        if(_joint_node.size() == 0)
	            continue;

            Dynamixel *_dxl = NULL;
	        std::map<std::string, Dynamixel*>::iterator _dxl_it = robot->dxls.find(_joint_name);
	        if(_dxl_it != robot->dxls.end())
	            _dxl = _dxl_it->second;

	        if(_dxl == NULL)
	        {
	            ROS_WARN("Joint [%s] does not found.", _joint_name.c_str());
	            continue;
	        }
	        if(DEBUG_PRINT) ROS_INFO("JOINT_NAME: %s", _joint_name.c_str());

	        for(YAML::const_iterator _it_joint = _joint_node.begin(); _it_joint != _joint_node.end(); _it_joint++)
	        {
	            std::string _item_name  = _it_joint->first.as<std::string>();

	            if(DEBUG_PRINT) ROS_INFO("  ITEM_NAME: %s", _item_name.c_str());
	            // indirect address setting
	            if(_item_name == INDIRECT_ADDRESS)
	            {
	                YAML::Node _indirect_node = _joint_node[_item_name];
	                if(_indirect_node.size() == 0)
	                    continue;

	                for(YAML::const_iterator _it_idx = _indirect_node.begin(); _it_idx != _indirect_node.end(); _it_idx++)
	                {
	                    int _start_idx = _it_idx->first.as<int>();
	                    if(_start_idx < 1 || 256 < _start_idx)
	                        ROS_WARN("[%s] INDIRECT ADDRESS start index is out of range. (%d)", _joint_name.c_str(), _start_idx);
	                    if(DEBUG_PRINT) ROS_INFO("    START_IDX: %d", _start_idx);

	                    YAML::Node _indirect_item_node = _indirect_node[_start_idx];
	                    if(_indirect_item_node.size() == 0)
	                        continue;

	                    int _start_address = _dxl->ctrl_table[INDIRECT_ADDRESS]->address + _start_idx - 1;
	                    for(unsigned int _i = 0; _i < _indirect_item_node.size(); _i++)
	                    {
	                        std::string _indir_item_name = _indirect_item_node[_i].as<std::string>().c_str();
	                        int _addr_leng = _dxl->ctrl_table[_indir_item_name]->data_length;
	                        for(int _l = 0; _l < _addr_leng; _l++)
	                        {
	                        	if(DEBUG_PRINT) ROS_INFO("      - INDIR_ADDR[%d] : %d", _start_address, _dxl->ctrl_table[_indir_item_name]->address + _l);
	                            Write2Byte(_joint_name, _start_address, _dxl->ctrl_table[_indir_item_name]->address + _l);
	                            _start_address += 2;
	                        }
	                    }
	                }
	            }
	            // other items setting
	            else
	            {
	                UINT32_T    _value      = _it_joint->second.as<UINT32_T>();

	                ControlTableItem *_item = _dxl->ctrl_table[_item_name];
	                if(_item == NULL)
	                {
	                    ROS_WARN("Control Item [%s] does not found.", _item_name.c_str());
	                    continue;
	                }

	                switch(_item->data_length)
	                {
	                case 1:
	                    Write1Byte(_joint_name, _item->address, (UINT8_T)_value);
	                    break;
	                case 2:
	                    Write2Byte(_joint_name, _item->address, (UINT16_T)_value);
	                    break;
	                case 4:
	                    Write4Byte(_joint_name, _item->address, _value);
	                    break;
	                default:
	                    break;
	                }
	            }
	        }
	    }
	} catch(const std::exception& e) {
	    ROS_INFO("Dynamixel Init file not found.");
	}


	// [ BulkRead ] StartAddress : Present Position , Length : 10 ( Position/Velocity/Current )
	for(std::map<std::string, Dynamixel*>::iterator _it = robot->dxls.begin(); _it != robot->dxls.end(); _it++)
	{
		std::string _joint_name = _it->first;
		Dynamixel  *_dxl        = _it->second;

        if(_dxl == NULL)
            continue;

		UINT16_T _data_length = 0;
		if(_dxl->id == 11 || _dxl->id == 12 || _dxl->id == 23 || _dxl->id == 24)
			_data_length = 17;
		else if(_dxl->id == 13 || _dxl->id == 14 || _dxl->id == 25 || _dxl->id == 26)
			_data_length = 21;
		else
			_data_length = 13;
		port_to_bulk_read[_dxl->port_name]->AddParam(robot->dxls[_joint_name]->id, PRESENT_POSITION_ADDR, _data_length);

		// Torque ON
		_protocol2_handler->Write1ByteTxRx(robot->ports[_dxl->port_name], robot->dxls[_joint_name]->id, TORQUE_ENABLE_ADDR, 1);
	}

	//InitSyncWrite();

	for(std::map<std::string, PortHandler *>::iterator _it = robot->ports.begin(); _it != robot->ports.end(); _it++)
	{
		// set goal velocity = 0
		_protocol2_handler->Write4ByteTxOnly(_it->second, BROADCAST_ID, GOAL_VELOCITY_ADDR, 0);
		// set goal acceleration = 0
		_protocol2_handler->Write4ByteTxOnly(_it->second, BROADCAST_ID, GOAL_ACCELERATION_ADDR, 0);
	}

	/* temporary code end */

	queue_thread_ = boost::thread(boost::bind(&RobotisController::QueueThread, this));
	return true;
}

void RobotisController::QueueThread()
{
	ros::NodeHandle     _ros_node;
	ros::CallbackQueue  _callback_queue;

	_ros_node.setCallbackQueue(&_callback_queue);

	/* subscriber */
	ros::Subscriber _sync_write_item_sub= _ros_node.subscribe("/robotis/sync_write_item", 10, &RobotisController::SyncWriteItemCallback, this);
	ros::Subscriber _ctrl_module_sub    = _ros_node.subscribe("/robotis/set_ctrl_module", 10, &RobotisController::SetCtrlModuleCallback, this);
	ros::Subscriber _controller_mode_sub= _ros_node.subscribe("/robotis/set_controller_mode", 10, &RobotisController::SetControllerModeCallback, this);
	ros::Subscriber _direct_control_sub = _ros_node.subscribe("/robotis/set_joint_states", 10, &RobotisController::SetJointStatesCallback, this);

	/* publisher */
	goal_joint_state_pub                = _ros_node.advertise<sensor_msgs::JointState>("/robotis/goal_joint_states", 10);
	present_joint_state_pub             = _ros_node.advertise<sensor_msgs::JointState>("/robotis/present_joint_states", 10);
	current_module_pub					= _ros_node.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/current_ctrl_module", 10);

	/* service */
	ros::ServiceServer _joint_module_server = _ros_node.advertiseService("/robotis/get_ctrl_module", &RobotisController::GetCtrlModuleCallback, this);

	while(_ros_node.ok())
	{
		_callback_queue.callAvailable();
	}
}

void *RobotisController::ThreadProc(void *param)
{
	RobotisController *controller = (RobotisController *)param;
	static struct timespec next_time;
	static struct timespec curr_time;

	ROS_INFO("controller::thread_proc");

	clock_gettime(CLOCK_MONOTONIC, &next_time);

	while(!controller->stop_timer_)
	{
		next_time.tv_sec   += (next_time.tv_nsec + CONTROL_CYCLE_MSEC * 1000000) / 1000000000;
		next_time.tv_nsec   = (next_time.tv_nsec + CONTROL_CYCLE_MSEC * 1000000) % 1000000000;

		controller->Process();

	    clock_gettime(CLOCK_MONOTONIC, &curr_time);
	    long delta_nsec = (next_time.tv_sec - curr_time.tv_sec) * 1000000000 + (next_time.tv_nsec - curr_time.tv_nsec);
	    if(delta_nsec < -100000 )
	    {
	        if(controller->DEBUG_PRINT == true)
	            ROS_WARN("[RobotisController::ThreadProc] NEXT TIME < CURR TIME.. (%f)[%ld.%09ld / %ld.%09ld]", delta_nsec/1000000.0, (long)next_time.tv_sec, (long)next_time.tv_nsec, (long)curr_time.tv_sec, (long)curr_time.tv_nsec);
	        // next_time = curr_time + 3 msec
	        next_time.tv_sec    = curr_time.tv_sec + (curr_time.tv_nsec + 3000000) / 1000000000;
	        next_time.tv_nsec   = (curr_time.tv_nsec + 3000000) % 1000000000;
	    }

		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
	}
	return 0;
}

void RobotisController::StartTimer()
{
	if(this->is_timer_running_ == true)
		return;

	InitSyncWrite();

    for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
        _it->second->TxPacket();

	int error;
	struct sched_param param;
	pthread_attr_t attr;

	pthread_attr_init(&attr);

	error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
	if(error != 0)
		ROS_ERROR("pthread_attr_setschedpolicy error = %d\n",error);
	error = pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
	if(error != 0)
		ROS_ERROR("pthread_attr_setinheritsched error = %d\n",error);

	memset(&param, 0, sizeof(param));
	param.sched_priority = 31;// RT
	error = pthread_attr_setschedparam(&attr, &param);
	if(error != 0)
		ROS_ERROR("pthread_attr_setschedparam error = %d\n",error);

	// create and start the thread
	if((error = pthread_create(&this->timer_thread_, &attr, this->ThreadProc, this))!= 0) {
		ROS_ERROR("timer thread create fail!!");
		exit(-1);
	}

	this->is_timer_running_ = true;
}

void RobotisController::StopTimer()
{
	int error = 0;

	// set the flag to stop the thread
	if(this->is_timer_running_)
	{
		this->stop_timer_ = true;

		// wait until the thread is stopped.
		if((error = pthread_join(this->timer_thread_, NULL)) != 0)
			exit(-1);

		for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
			_it->second->RxPacket();

		for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write.begin(); _it != port_to_sync_write.end(); _it++)
			_it->second->ClearParam();

		this->stop_timer_ = false;
		this->is_timer_running_ = false;
	}
}

bool RobotisController::IsTimerRunning()
{
	return this->is_timer_running_;
}

void RobotisController::LoadOffset(const std::string path)
{
	YAML::Node _doc;
	try{
		_doc = YAML::LoadFile(path.c_str());
	} catch(const std::exception& e) {
		ROS_ERROR("Fail to load offset yaml.");
		return;
	}

	YAML::Node _offset_node = _doc["offset"];
	if(_offset_node.size() == 0)
		return;

	ROS_INFO("Load offsets...");
	for(YAML::const_iterator _it = _offset_node.begin(); _it != _offset_node.end(); _it++)
	{
		std::string _joint_name = _it->first.as<std::string>();
		double      _offset     = _it->second.as<double>();

		std::map<std::string, Dynamixel*>::iterator _dxl_it = robot->dxls.find(_joint_name);
		if(_dxl_it != robot->dxls.end())
		    _dxl_it->second->dxl_state->position_offset = _offset;
	}
}

void RobotisController::Process()
{
	static bool _is_process_running = false;

	if(_is_process_running == true)
		return;
	_is_process_running = true;

	// ROS_INFO("Controller::Process()");

	sensor_msgs::JointState _present_state;
	sensor_msgs::JointState _goal_state;

	ros::Time _now = ros::Time::now();


	// TODO: BulkRead Rx
	bool _do_sync_write = false;

	for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
		_it->second->RxPacket();

    ros::Duration _dur = ros::Time::now() - _now;
    double _msec = _dur.nsec * 0.000001;

    if(_msec > 8) std::cout << "Process duration  : " << _msec << std::endl;

	// -> save to Robot->dxls[]->dynamixel_state.present_position
	// -> update time stamp to Robot->dxls[]->dynamixel_state.update_time_stamp
	for(std::map<std::string, Dynamixel *>::iterator dxl_it = robot->dxls.begin(); dxl_it != robot->dxls.end(); dxl_it++)
	{
		UINT32_T _pos = 0;

		std::string _joint_name = dxl_it->first;
		Dynamixel *_dxl = dxl_it->second;

		if(port_to_bulk_read[_dxl->port_name]->GetData(_dxl->id, PRESENT_POSITION_ADDR, &_pos) == true)
			_dxl->dxl_state->present_position = _dxl->ConvertValue2Radian(_pos) - _dxl->dxl_state->position_offset; // remove offset

		UINT16_T _ext_data = 0;
		if(_dxl->id == 11 || _dxl->id == 12 || _dxl->id == 23 || _dxl->id == 24)
		{
			if(port_to_bulk_read[_dxl->port_name]->GetData(_dxl->id, EXT_PORT_DATA1_ADDR, &_ext_data) == true)
				_dxl->dxl_state->ext_port_data[0] = _ext_data;
			if(port_to_bulk_read[_dxl->port_name]->GetData(_dxl->id, EXT_PORT_DATA2_ADDR, &_ext_data) == true)
				_dxl->dxl_state->ext_port_data[1] = _ext_data;
		}
		else if(_dxl->id == 13 || _dxl->id == 14 || _dxl->id == 25 || _dxl->id == 26)
		{
			if(port_to_bulk_read[_dxl->port_name]->GetData(_dxl->id, EXT_PORT_DATA1_ADDR, &_ext_data) == true)
				_dxl->dxl_state->ext_port_data[0] = _ext_data;
			if(port_to_bulk_read[_dxl->port_name]->GetData(_dxl->id, EXT_PORT_DATA2_ADDR, &_ext_data) == true)
				_dxl->dxl_state->ext_port_data[1] = _ext_data;
			if(port_to_bulk_read[_dxl->port_name]->GetData(_dxl->id, EXT_PORT_DATA3_ADDR, &_ext_data) == true)
				_dxl->dxl_state->ext_port_data[2] = _ext_data;
			if(port_to_bulk_read[_dxl->port_name]->GetData(_dxl->id, EXT_PORT_DATA4_ADDR, &_ext_data) == true)
				_dxl->dxl_state->ext_port_data[3] = _ext_data;
		}

		_present_state.name.push_back(_joint_name);
		// TODO: should be converted to rad, rad/s, Nm
		_present_state.position.push_back(_dxl->dxl_state->present_position);
		_present_state.velocity.push_back(_dxl->dxl_state->present_velocity);
		_present_state.effort.push_back(_dxl->dxl_state->present_load);

        _goal_state.name.push_back(_joint_name);
        _goal_state.position.push_back(_dxl->dxl_state->goal_position);
        _goal_state.velocity.push_back(_dxl->dxl_state->goal_velocity);
        _goal_state.effort.push_back(_dxl->dxl_state->goal_torque);
	}
	// -> publish present joint_states topic
	_present_state.header.stamp = ros::Time::now();
	present_joint_state_pub.publish(_present_state);

    // -> publish goal joint_states topic
    _goal_state.header.stamp = _present_state.header.stamp;
    goal_joint_state_pub.publish(_goal_state);

	if(controller_mode_ == MOTION_MODULE_MODE)
	{
		// TODO: Call MotionModule Process()
		// -> for loop : call MotionModule list -> Process()
		if(modules_.size() > 0)
		{
			queue_mutex_.lock();

			for(std::list<MotionModule*>::iterator module_it = modules_.begin(); module_it != modules_.end(); module_it++)
			{
				// ros::Time _before = ros::Time::now();

				(*module_it)->Process(robot->dxls);

				// ros::Duration _dur = ros::Time::now() - _before;
				// double _msec = _dur.nsec * 0.000001;

				// std::cout << "Process duration ["<< (*module_it)->module_name << "] : " << _msec << std::endl;

				// for loop : joint list
				for(std::map<std::string, Dynamixel *>::iterator dxl_it = robot->dxls.begin(); dxl_it != robot->dxls.end(); dxl_it++)
				{
					std::string _joint_name = dxl_it->first;
					Dynamixel *_dxl = dxl_it->second;

					DynamixelState *_dxl_state = _dxl->dxl_state;
					if(_dxl->ctrl_module_name == (*module_it)->module_name)
					{
						_do_sync_write = true;
						// ROS_INFO("Set goal value");
						DynamixelState *_result_state = (*module_it)->result[_joint_name];

						if(_result_state == NULL) {
							ROS_INFO("[%s] %s", (*module_it)->module_name.c_str(), _joint_name.c_str());
							continue;
						}

						// TODO: check update time stamp ?

						if((*module_it)->control_mode == POSITION_CONTROL)
						{
//							if(_result_state->goal_position == 0 && _dxl->id == 3)
//								ROS_INFO("[MODULE:%s][ID:3] goal position = %f", (*module_it)->module_name.c_str(), _dxl_state->goal_position);
							_dxl_state->goal_position = _result_state->goal_position;

							// add offset
							UINT32_T _pos_data = _dxl->ConvertRadian2Value(_dxl_state->goal_position + _dxl_state->position_offset);
							UINT8_T _sync_write_data[4];
							_sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos_data));
							_sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos_data));
							_sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos_data));
							_sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos_data));

							if(abs(_pos_data) > 151800)
								printf("goal_pos : %f |  position_offset : %f | pos_data : %d\n", _dxl_state->goal_position , _dxl_state->position_offset, _pos_data);

							port_to_sync_write[_dxl->port_name]->ChangeParam(_dxl->id, _sync_write_data);
						}
						else if((*module_it)->control_mode == VELOCITY_CONTROL)
						{
							_dxl_state->goal_velocity = _result_state->goal_velocity;
						}
						else if((*module_it)->control_mode == TORQUE_CONTROL)
						{
							_dxl_state->goal_torque = _result_state->goal_torque;
						}
					}
				}
			}

			// std::cout << " ------------------------------------------- " << std::endl;
			queue_mutex_.unlock();
		}

		// TODO: Combine the result && SyncWrite
		// -> SyncWrite
		if(_do_sync_write) {
			for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write.begin(); _it != port_to_sync_write.end(); _it++)
				_it->second->TxPacket();
		}
	}
	else if(controller_mode_ == DIRECT_CONTROL_MODE)
	{
		queue_mutex_.lock();

		for(std::map<std::string, GroupSyncWrite *>::iterator _it = port_to_sync_write.begin(); _it != port_to_sync_write.end(); _it++)
		{
			_it->second->TxPacket();
			_it->second->ClearParam();
		}

		if(direct_sync_write_.size() > 0)
		{
			for(int _i = 0; _i < direct_sync_write_.size(); _i++)
			{
				direct_sync_write_[_i]->TxPacket();
				direct_sync_write_[_i]->ClearParam();
			}
			direct_sync_write_.clear();
		}

		queue_mutex_.unlock();
	}

	// TODO: User Read/Write

	// TODO: BulkRead Tx
	for(std::map<std::string, GroupBulkRead *>::iterator _it = port_to_bulk_read.begin(); _it != port_to_bulk_read.end(); _it++)
		_it->second->TxPacket();

	// for test : goal to present
	//	for(std::map<std::string, Dynamixel *>::iterator dxl_it = robot_->dxls.begin(); dxl_it != robot_->dxls.end(); dxl_it++)
	//	{
	//		dxl_it->second->dxl_state->present_position = dxl_it->second->dxl_state->goal_position;
	//		dxl_it->second->dxl_state->present_velocity = dxl_it->second->dxl_state->goal_velocity;
	//	}


//		ros::Duration _dur = ros::Time::now() - _now;
//		double _msec = _dur.nsec * 0.000001;
//
//		if(_msec > 8) std::cout << "Process duration  : " << _msec << std::endl;

	_is_process_running = false;
}

void RobotisController::AddModule(MotionModule *module)
{
	module->Initialize(CONTROL_CYCLE_MSEC);
	modules_.push_back(module);
	modules_.unique();
}

void RobotisController::RemoveModule(MotionModule *module)
{
	modules_.remove(module);
}

void RobotisController::SyncWriteItemCallback(const robotis_controller_msgs::SyncWriteItem::ConstPtr &msg)
{
	for(int _i = 0; _i < msg->joint_name.size(); _i++)
	{
		Dynamixel           *_dxl           = robot->dxls[msg->joint_name[_i]];
		ControlTableItem    *_item          = _dxl->ctrl_table[msg->item_name];

		PortHandler         *_port          = robot->ports[_dxl->port_name];
		PacketHandler       *_packet_handler= PacketHandler::GetPacketHandler(_dxl->protocol_version);

		if(_item->access_type == READ)
		    continue;

		int _idx = 0;
		if(direct_sync_write_.size() == 0)
		{
			direct_sync_write_.push_back(new GroupSyncWrite(_port, _packet_handler, _item->address, _item->data_length));
			_idx = 0;
		}
		else
		{
			for(_idx = 0; _idx < direct_sync_write_.size(); _idx++)
			{
				if(direct_sync_write_[_idx]->GetPortHandler() == _port &&
						direct_sync_write_[_idx]->GetPacketHandler() == _packet_handler)
					break;
			}

			if(_idx == direct_sync_write_.size())
				direct_sync_write_.push_back(new GroupSyncWrite(_port, _packet_handler, _item->address, _item->data_length));
		}

		UINT8_T *_data = new UINT8_T[_item->data_length];
		if(_item->data_length == 1)
			_data[0] = (UINT8_T)msg->value[_i];
		else if(_item->data_length == 2)
		{
			_data[0] = DXL_LOBYTE((UINT16_T)msg->value[_i]);
			_data[1] = DXL_HIBYTE((UINT16_T)msg->value[_i]);
		}
		else if(_item->data_length == 4)
		{
			_data[0] = DXL_LOBYTE(DXL_LOWORD((UINT32_T)msg->value[_i]));
			_data[1] = DXL_HIBYTE(DXL_LOWORD((UINT32_T)msg->value[_i]));
			_data[2] = DXL_LOBYTE(DXL_HIWORD((UINT32_T)msg->value[_i]));
			_data[3] = DXL_HIBYTE(DXL_HIWORD((UINT32_T)msg->value[_i]));
		}
		direct_sync_write_[_idx]->AddParam(_dxl->id, _data);
		delete[] _data;
	}
}

void RobotisController::SetControllerModeCallback(const std_msgs::String::ConstPtr &msg)
{
	if(msg->data == "DIRECT_CONTROL_MODE")
		controller_mode_ = DIRECT_CONTROL_MODE;
	else if(msg->data == "MOTION_MODULE_MODE")
		controller_mode_ = MOTION_MODULE_MODE;
}

void RobotisController::SetJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	if(controller_mode_ != DIRECT_CONTROL_MODE)
		return;

	queue_mutex_.lock();

	for(int _i = 0; _i < msg->name.size(); _i++)
	{
		INT32_T _pos = 0;

		Dynamixel *_dxl = robot->dxls[msg->name[_i]];
		if(_dxl == NULL)
			continue;

		_dxl->dxl_state->goal_position = msg->position[_i];
		_pos = _dxl->ConvertRadian2Value((double)msg->position[_i]);

		UINT8_T _sync_write_data[4];
		_sync_write_data[0] = DXL_LOBYTE(DXL_LOWORD(_pos));
		_sync_write_data[1] = DXL_HIBYTE(DXL_LOWORD(_pos));
		_sync_write_data[2] = DXL_LOBYTE(DXL_HIWORD(_pos));
		_sync_write_data[3] = DXL_HIBYTE(DXL_HIWORD(_pos));

		port_to_sync_write[_dxl->port_name]->AddParam(_dxl->id, _sync_write_data);
	}

	queue_mutex_.unlock();
}

void RobotisController::SetCtrlModuleCallback(const robotis_controller_msgs::JointCtrlModule::ConstPtr &msg)
{
	if(msg->joint_name.size() != msg->module_name.size())
		return;

	queue_mutex_.lock();

	for(unsigned int idx = 0; idx < msg->joint_name.size(); idx++)
	{
	    Dynamixel *_dxl = NULL;
        std::map<std::string, Dynamixel*>::iterator _dxl_it = robot->dxls.find((std::string)(msg->joint_name[idx]));
        if(_dxl_it != robot->dxls.end())
            _dxl = _dxl_it->second;
        else
            continue;

		// none
		if(msg->module_name[idx] == "" || msg->module_name[idx] == "none")
		{
		    _dxl->ctrl_module_name = msg->module_name[idx];
			continue;
		}

		// check whether the module is using this joint
		for(std::list<MotionModule *>::iterator _m_it = modules_.begin(); _m_it != modules_.end(); _m_it++)
		{
			if((*_m_it)->module_name == msg->module_name[idx])
			{
				if((*_m_it)->result.find(msg->joint_name[idx]) != (*_m_it)->result.end())
				{
				    _dxl->ctrl_module_name = msg->module_name[idx];
					break;
				}
			}
		}
	}

	for(std::list<MotionModule *>::iterator _m_it = modules_.begin(); _m_it != modules_.end(); _m_it++)
	{
		// set all modules -> disable
		(*_m_it)->enable = false;

		// set all used modules -> enable
		for(std::map<std::string, Dynamixel*>::iterator _d_it = robot->dxls.begin(); _d_it != robot->dxls.end(); _d_it++)
		{
			if(_d_it->second->ctrl_module_name == (*_m_it)->module_name)
			{
				(*_m_it)->enable = true;
				break;
			}
		}
	}

	// TODO: set indirect address
	// -> check module's control_mode

	queue_mutex_.unlock();

	robotis_controller_msgs::JointCtrlModule _current_module_msg;
	for(std::map<std::string, Dynamixel *>::iterator _dxl_iter = robot->dxls.begin(); _dxl_iter  != robot->dxls.end(); ++_dxl_iter)
	{
		_current_module_msg.joint_name.push_back(_dxl_iter->first);
		_current_module_msg.module_name.push_back(_dxl_iter->second->ctrl_module_name);
	}

	if(_current_module_msg.joint_name.size() == _current_module_msg.module_name.size())
		current_module_pub.publish(_current_module_msg);
}

bool RobotisController::GetCtrlModuleCallback(robotis_controller_msgs::GetJointModule::Request &req, robotis_controller_msgs::GetJointModule::Response &res)
{
	for(unsigned int idx = 0; idx < req.joint_name.size(); idx++)
	{
		std::map<std::string, Dynamixel*>::iterator _d_it = robot->dxls.find((std::string)(req.joint_name[idx]));
		if(_d_it != robot->dxls.end())
		{
			res.joint_name.push_back(req.joint_name[idx]);
			res.module_name.push_back(_d_it->second->ctrl_module_name);
		}
	}

	if(res.joint_name.size() == 0) return false;

	return true;
}

bool RobotisController::CheckTimerStop()
{
	if(this->is_timer_running_)
	{
		if(DEBUG_PRINT == true)
			ROS_WARN("Process Timer is running.. STOP the timer first.");
		return false;
	}
	return true;
}

int RobotisController::Ping(const std::string joint_name, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Ping(_port_handler, _dxl->id, error);
}
int RobotisController::Ping(const std::string joint_name, UINT16_T* model_number, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Ping(_port_handler, _dxl->id, model_number, error);
}

int RobotisController::Action(const std::string joint_name)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Action(_port_handler, _dxl->id);
}
int RobotisController::Reboot(const std::string joint_name, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Reboot(_port_handler, _dxl->id, error);
}
int RobotisController::FactoryReset(const std::string joint_name, UINT8_T option, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->FactoryReset(_port_handler, _dxl->id, option, error);
}

int RobotisController::Read(const std::string joint_name, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->ReadTxRx(_port_handler, _dxl->id, address, length, data, error);
}

int RobotisController::ReadCtrlItem(const std::string joint_name, const std::string item_name, UINT32_T *data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    ControlTableItem*_item          = _dxl->ctrl_table[item_name];
    if(_item == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    int _result = COMM_NOT_AVAILABLE;
    switch(_item->data_length)
    {
        case 1:
        {
            UINT8_T _data = 0;
            _result = _pkt_handler->Read1ByteTxRx(_port_handler, _dxl->id, _item->address, &_data, error);
            if(_result == COMM_SUCCESS)
                *data = _data;
            break;
        }
        case 2:
        {
            UINT16_T _data = 0;
            _result = _pkt_handler->Read2ByteTxRx(_port_handler, _dxl->id, _item->address, &_data, error);
            if(_result == COMM_SUCCESS)
                *data = _data;
            break;
        }
        case 4:
        {
            UINT32_T _data = 0;
            _result = _pkt_handler->Read4ByteTxRx(_port_handler, _dxl->id, _item->address, &_data, error);
            if(_result == COMM_SUCCESS)
                *data = _data;
            break;
        }
        default:
            break;
    }
    return _result;
}

int RobotisController::Read1Byte(const std::string joint_name, UINT16_T address, UINT8_T *data, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Read1ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Read2Byte(const std::string joint_name, UINT16_T address, UINT16_T *data, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Read2ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Read4Byte(const std::string joint_name, UINT16_T address, UINT32_T *data, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Read4ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Write(const std::string joint_name, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->WriteTxRx(_port_handler, _dxl->id, address, length, data, error);
}

int RobotisController::WriteCtrlItem(const std::string joint_name, const std::string item_name, UINT32_T data, UINT8_T *error)
{
    if(CheckTimerStop() == false)
        return COMM_PORT_BUSY;

    Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

    ControlTableItem*_item          = _dxl->ctrl_table[item_name];
    if(_item == NULL)
        return COMM_NOT_AVAILABLE;

    PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
    PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

    int _result = COMM_NOT_AVAILABLE;
    UINT8_T *_data = new UINT8_T[_item->data_length];
    if(_item->data_length == 1)
    {
        _data[0] = (UINT8_T)data;
        _result = _pkt_handler->Write1ByteTxRx(_port_handler, _dxl->id, _item->address, data, error);
    }
    else if(_item->data_length == 2)
    {
        _data[0] = DXL_LOBYTE((UINT16_T)data);
        _data[1] = DXL_HIBYTE((UINT16_T)data);
        _result = _pkt_handler->Write2ByteTxRx(_port_handler, _dxl->id, _item->address, data, error);
    }
    else if(_item->data_length == 4)
    {
        _data[0] = DXL_LOBYTE(DXL_LOWORD((UINT32_T)data));
        _data[1] = DXL_HIBYTE(DXL_LOWORD((UINT32_T)data));
        _data[2] = DXL_LOBYTE(DXL_HIWORD((UINT32_T)data));
        _data[3] = DXL_HIBYTE(DXL_HIWORD((UINT32_T)data));
        _result = _pkt_handler->Write4ByteTxRx(_port_handler, _dxl->id, _item->address, data, error);
    }
    delete[] _data;
    return _result;
}

int RobotisController::Write1Byte(const std::string joint_name, UINT16_T address, UINT8_T data, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Write1ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Write2Byte(const std::string joint_name, UINT16_T address, UINT16_T data, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Write2ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::Write4Byte(const std::string joint_name, UINT16_T address, UINT32_T data, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->Write4ByteTxRx(_port_handler, _dxl->id, address, data, error);
}

int RobotisController::RegWrite(const std::string joint_name, UINT16_T address, UINT16_T length, UINT8_T *data, UINT8_T *error)
{
	if(CheckTimerStop() == false)
		return COMM_PORT_BUSY;

	Dynamixel       *_dxl           = robot->dxls[joint_name];
    if(_dxl == NULL)
        return COMM_NOT_AVAILABLE;

	PacketHandler   *_pkt_handler   = PacketHandler::GetPacketHandler(_dxl->protocol_version);
	PortHandler     *_port_handler  = robot->ports[_dxl->port_name];

	return _pkt_handler->RegWriteTxRx(_port_handler, _dxl->id, address, length, data, error);
}

