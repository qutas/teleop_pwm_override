#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <teleop_pwm_override/PWMOverrideConfig.h>

#include <mavros_msgs/OverrideRCIn.h>

#include <math.h>
#include <vector>

class TeleopPWMOverride {
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle nhp_;
		ros::Timer timer_;

		ros::Publisher pub_pwm_;
		dynamic_reconfigure::Server<teleop_pwm_override::PWMOverrideConfig> dyncfg_pwm_;

		double param_rate_;
		int param_failsafe_pwm_;

		bool param_override_enable_;
		bool param_failsafe_enable_;
		int param_num_channels_;

		std::vector<bool> chan_enable_;
		std::vector<uint16_t> chan_raw_;

	public:
		TeleopPWMOverride( void );

		~TeleopPWMOverride( void );

	private:
		void callback_timer(const ros::TimerEvent& e);
		void callback_cfg_pwm(teleop_pwm_override::PWMOverrideConfig &config, uint32_t level);
};



TeleopPWMOverride::TeleopPWMOverride() :
	nh_(),
	nhp_("~"),
	param_rate_(20),
	param_override_enable_(false),
	param_failsafe_enable_(true),
	param_failsafe_pwm_(1000),
	param_num_channels_(8),
	dyncfg_pwm_(ros::NodeHandle(nh_, "pwm_override")) {

	nhp_.param("rate", param_rate_, param_rate_);
	nhp_.param("pwm_failsafe", param_failsafe_pwm_, param_failsafe_pwm_);
	chan_enable_.reserve(param_num_channels_);
	chan_raw_.reserve(param_num_channels_);

	pub_pwm_ = nhp_.advertise<mavros_msgs::OverrideRCIn>("pwm", 10);

	dyncfg_pwm_.setCallback(boost::bind(&TeleopPWMOverride::callback_cfg_pwm, this, _1, _2));

	timer_ = nh_.createTimer(ros::Duration(0.01), &TeleopPWMOverride::callback_timer, this);

	ROS_INFO("Ready to send PWM overrides");
}

TeleopPWMOverride::~TeleopPWMOverride() {
}


void TeleopPWMOverride::callback_cfg_pwm(teleop_pwm_override::PWMOverrideConfig &config, uint32_t level) {
	param_override_enable_ = config.override_enable;
	param_failsafe_enable_ = config.failsafe_enable;

	chan_enable_[0] = config.chan1_enable;
	chan_enable_[1] = config.chan2_enable;
	chan_enable_[2] = config.chan3_enable;
	chan_enable_[3] = config.chan4_enable;
	chan_enable_[4] = config.chan5_enable;
	chan_enable_[5] = config.chan6_enable;
	chan_enable_[6] = config.chan7_enable;
	chan_enable_[7] = config.chan8_enable;

	chan_raw_[0] = config.chan1_raw;
	chan_raw_[1] = config.chan2_raw;
	chan_raw_[2] = config.chan3_raw;
	chan_raw_[3] = config.chan4_raw;
	chan_raw_[4] = config.chan5_raw;
	chan_raw_[5] = config.chan6_raw;
	chan_raw_[6] = config.chan7_raw;
	chan_raw_[7] = config.chan8_raw;
}

void TeleopPWMOverride::callback_timer(const ros::TimerEvent& e) {
	mavros_msgs::OverrideRCIn msg_out;

	for(int i=0; i<param_num_channels_; i++) {
		if(param_override_enable_ && chan_enable_[i]) {
			if(param_failsafe_enable_) {
				msg_out.channels[i] = param_failsafe_pwm_;
			} else {
				msg_out.channels[i] = chan_raw_[i];
			}
		} else {
			msg_out.channels[i] = msg_out.CHAN_NOCHANGE;
		}
	}

	pub_pwm_.publish(msg_out);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "teleop_pwm_override");
	TeleopPWMOverride pwm;

	ros::spin();

	return 0;
}
