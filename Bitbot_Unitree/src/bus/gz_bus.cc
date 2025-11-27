#include "bus/gz_bus.h"
#include "bus/motor_crc_hg.h"

namespace bitbot {
	GzBus::GzBus() {
	}

	GzBus::~GzBus() {
		this->PowerOff();
		this->WriteBus();
		this->logger_->info("GzBus shutdown.");
	}

	void GzBus::PowerOn()
	{
		for (auto dev : this->joint_devices_)
		{
			auto dev_ = dynamic_cast<GzJoint*>(dev);
			dev_->PowerOn();
		}
	}

	void GzBus::PowerOff()
	{
		for (auto dev : this->joint_devices_)
		{
			auto dev_ = dynamic_cast<GzJoint*>(dev);
			dev_->PowerOff();
		}
	}

	void GzBus::InitPublishersAndSubscribers()
	{
		using namespace unitree::robot;
		using namespace unitree_hg::msg::dds_;
		//TODO: 研究一下msc是什么
		this->low_state_subscriber_.reset(new ChannelSubscriber<LowState_>(LOW_STATE_TOPIC));
		this->low_state_subscriber_->InitChannel(std::bind(&GzBus::LowStateCallback, this, std::placeholders::_1), 1);

		this->alter_imu_subscriber_.reset(new ChannelSubscriber<IMUState_>(ALTER_IMU_STATE_TOPIC));
		this->alter_imu_subscriber_->InitChannel(std::bind(&GzBus::AlterImuCallback, this, std::placeholders::_1), 1);

		this->main_board_state_subscriber_.reset(new ChannelSubscriber<MainBoardState_>(MAINBOARD_STATE_TOPIC));
		this->main_board_state_subscriber_->InitChannel(std::bind(&GzBus::MainBoardStateCallback, this, std::placeholders::_1), 1);

		this->bms_state_subscriber_.reset(new ChannelSubscriber<BmsState_>(BMS_STATE_TOPIC));
		this->bms_state_subscriber_->InitChannel(std::bind(&GzBus::BmsStateCallback, this, std::placeholders::_1), 1);

		this->low_command_publisher_.reset(new ChannelPublisher<LowCmd_>(LOW_CMD_TOPIC));
		this->low_command_publisher_->InitChannel();
	}


	void GzBus::Init(pugi::xml_node& bus_node)
	{
		std::string mode_pr_str;
		ConfigParser::ParseAttribute2s(mode_pr_str, bus_node.attribute("mode_pr"));
		if (mode_pr_str == "PR")
		{
			this->mode_pr_ = 0;
		}
		else if (mode_pr_str == "AB")
		{
			this->mode_pr_ = 1;
		}
		else
		{
			this->logger_->error("GzBus Init error: mode_pr attribute invalid: {}", mode_pr_str);
			throw std::runtime_error("GzBus Init error: mode_pr attribute invalid");
		}

		std::string mode_machine_str;
		ConfigParser::ParseAttribute2s(mode_machine_str, bus_node.attribute("mode_machine"));
		if (mode_machine_str == "23DoF")
		{
			this->mode_machine_ = 4;
		}
		else if (mode_machine_str == "29DoF")
		{
			this->mode_machine_ = 5;
		}
		else if (mode_machine_str == "27DoF")
		{
			this->mode_machine_ = 6;
		}
		else
		{
			this->logger_->error("GzBus Init error: mode_machine attribute invalid: {}", mode_machine_str);
			throw std::runtime_error("GzBus Init error: mode_machine attribute invalid");
		}


		for (auto dev : this->devices_)
		{
			if (dev->Type() == (uint32_t)GzDeviceType::GZ_JOINT)
			{
				this->joint_devices_.push_back(dev);
			}
			else if (dev->Type() == (uint32_t)GzDeviceType::GZ_IMU)
			{
				this->imu_devices_.push_back(dev);
			}
		}

		if (this->joint_devices_.size() < 29)
		{
			this->logger_->error("GzBus Init error: joint devices size is not 35, actual size is {}", this->joint_devices_.size());
			throw std::runtime_error("GzBus Init error: joint devices size is not 35");
		}
		if (this->imu_devices_.size() != 2)
		{
			this->logger_->error("GzBus Init error: imu devices size is not 2, actual size is {}", this->imu_devices_.size());
			throw std::runtime_error("GzBus Init error: imu devices size is not 2");
		}

		std::string NetWorkCardName;
		ConfigParser::ParseAttribute2s(NetWorkCardName, bus_node.attribute("NetWorkCardName"));
		this->logger_->info("DDSNetWorkCardName: {}", NetWorkCardName);
		unitree::robot::ChannelFactory::Instance()->Init(0, NetWorkCardName);
		//init publishers and subscribers
		this->InitPublishersAndSubscribers();
	}

	void GzBus::RegisterDevices() {
		static DeviceRegistrar<GzDevice, GzJoint> gz_joint((uint32_t)GzDeviceType::GZ_JOINT, "GzJoint");
		static DeviceRegistrar<GzDevice, GzImu> gz_imu((uint32_t)GzDeviceType::GZ_IMU, "GzImu");
	}

	void GzBus::WriteBus() {
		this->motor_cmd_lock_.lock();
		unitree_hg::msg::dds_::LowCmd_ low_cmd_msg;
		for (size_t i = 0; i < this->joint_devices_.size(); ++i) {
			low_cmd_msg.motor_cmd()[i] = std::get<unitree_hg::msg::dds_::MotorCmd_>(this->joint_devices_[i]->Output());
		}
		this->motor_cmd_lock_.unlock();
		low_cmd_msg.mode_pr() = this->mode_pr_;
		low_cmd_msg.mode_machine() = this->mode_machine_;

		get_crc(low_cmd_msg);
		//publish low command message
		this->low_command_publisher_->Write(low_cmd_msg);
	}

	void GzBus::ReadBus() {
		this->motor_state_lock_.lock();
		for (size_t i = 0; i < this->joint_devices_.size(); ++i) {
			this->joint_devices_[i]->Input(this->motor_states_[i]);
		}
		this->motor_state_lock_.unlock();
		this->imu_state_lock_.lock();
		for (size_t i = 0; i < 2; ++i) {
			this->imu_devices_[i]->Input(this->imu_states_[i]);
		}
		this->imu_state_lock_.unlock();

		this->main_board_state_lock_.lock();
		// this->motherboard_device_.Input(this->main_board_state_msg_);
		this->main_board_state_lock_.unlock();

		this->bms_state_lock_.lock();
		// this->battery_device_.Input(this->bms_state_msg_);
		this->bms_state_lock_.unlock();
	}

	void GzBus::LowStateCallback(const void* msg_)
	{
		auto msg = static_cast<const unitree_hg::msg::dds_::LowState_*>(msg_);
		this->motor_state_lock_.lock();
		for (size_t i = 0; i < this->joint_devices_.size(); ++i) {
			this->motor_states_[i] = msg->motor_state()[i];
		}
		this->motor_state_lock_.unlock();

		this->imu_state_lock_.lock();
		this->imu_states_[0] = msg->imu_state();
		this->imu_state_lock_.unlock();

		msg->mode_pr() != this->mode_pr_ ? this->logger_->warn("LowStateCallback: mode_pr mismatch! received: {}, expected: {}", msg->mode_pr(), this->mode_pr_) : void();
		msg->mode_machine() != this->mode_machine_ ? this->logger_->warn("LowStateCallback: mode_machine mismatch! received: {}, expected: {}", msg->mode_machine(), this->mode_machine_) : void();

		//TODO: add remote and other states
		this->received.store(true);
	}

	void GzBus::AlterImuCallback(const void* msg_)
	{
		auto msg = static_cast<const unitree_hg::msg::dds_::IMUState_*>(msg_);
		this->imu_state_lock_.lock();
		this->imu_states_[1] = *msg;
		this->imu_state_lock_.unlock();

	}

	void GzBus::MainBoardStateCallback(const void* msg_)
	{
		auto msg = static_cast<const unitree_hg::msg::dds_::MainBoardState_*>(msg_);
		this->main_board_state_lock_.lock();
		this->main_board_state_msg_ = *msg;
		this->main_board_state_lock_.unlock();
	}

	void GzBus::BmsStateCallback(const void* msg_)
	{
		auto msg = static_cast<const unitree_hg::msg::dds_::BmsState_*>(msg_);
		this->bms_state_lock_.lock();
		this->bms_state_msg_ = *msg;
		this->bms_state_lock_.unlock();
	}

}
