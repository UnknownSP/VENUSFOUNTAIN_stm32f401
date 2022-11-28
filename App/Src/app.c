#include "app.h"
#include "DD_Gene.h"
#include "DD_RCDefinition.h"
#include "SystemTaskManager.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "MW_GPIO.h"
#include "MW_IWDG.h"
#include "message.h"
#include "MW_flash.h"
#include "constManager.h"

static
int bumper_setting(int num, bool oneshot, bool enable);
static
int bumper_medal_get(bool recet);
static
int outblock_setting(int side, int num, bool enable, bool stand, bool sit);
static 
int center_motor_setting(Motor_direction_t direction, uint8_t duty);
static
int outblock_get_status(int side, int num);
static
int JPrift_get_status(int side, int num);
static
int outball_get_status(int side, int num);
static
int outball_get_num(int side);
static 
BallLaunch_t ball_launch(int side, bool reset);
static
int launchball_set(int side, bool reset);
static
bool JPC_get_status(int side, int *num);
static 
int JPlottery_motor_setting(Motor_direction_t direction, uint8_t duty, int side);
static 
int JPrift_motor_setting(Motor_direction_t direction, uint8_t duty, int side);
static
bool JPrift_set(int side, int target_posi, bool reset);

static
int getRotateDirectionByMech(int recentPocket, int nowPocket, int nowDirection);

static int win_medal_coef = 2;
volatile uint8_t Pocket_Number;
volatile double Pocket_Number_Detailed;
volatile uint8_t Pocket_In_Number;

int appInit(void){

	ad_init();

	/*GPIO の設定などでMW,GPIOではHALを叩く*/
	return EXIT_SUCCESS;
}

/*application tasks*/
int appTask(void){
	
	int i;
	static unsigned int win_medal = 0;

	static bool enable_flag = false;
	static bool button_flag = true;
	static unsigned int test_duty = 0;
	static unsigned int rift_target = 0;
	static unsigned int JPC_in_num = 0;
	static bool JPC_flag = true;
	static int test_JPC_duty = 0;
	static int button_count = 0;
	static bool _is_manual = false;
	static bool _launch_R_Ball = false;
	static bool _launch_R_S1 = false;
	static bool _launch_R_S2 = false;
	static bool _launch_L_Ball = false;
	static bool _launch_L_S1 = false;
	static bool _launch_L_S2 = false;
	static bool _sens_pos_1 = false;
	static bool _sens_pos_2 = false;
	static bool _sens_pos_3 = false;
	static bool _sens_pos_4 = false;
	static bool _sens_pos_5 = false;
	static bool _sens_pos_1_enable = true;
	int pocket = 0;
	static bool _ball_InStop = false;
	static bool _ball_InStopped = false;
	static int rotate_direction = 1;
	static int rotate_direction_byMech = 1;
	static bool _Pos_control = false;

	if(MW_GPIORead(GPIOCID,GPIO_PIN_13) == 0 & button_flag){
		button_flag = false;
		button_count++;
		if(!enable_flag){
			enable_flag = true;
		}else{
			//enable_flag = false;
		}
	}
	if(MW_GPIORead(GPIOCID,GPIO_PIN_13) == 1){
		button_flag = true;
	}

	if(((PC_control_rcv[0] & 0b10000000) >> 7) == 1){
		enable_flag = true;
	}else{
		enable_flag = false;
	}
	if(((PC_control_rcv[0] & 0b00100000) >> 5) == 1){
		_is_manual = true;
	}else{
		_is_manual = false;
	}
	if(((PC_control_rcv[1] & 0b10000000) >> 7) == 1){
		_ball_InStop = true;
	}else{
		_ball_InStop = false;
	}
	if(((PC_control_rcv[3] & 0b10000000) >> 7) == 1){
		_Pos_control = true;
	}else{
		_Pos_control = false;
	}
	if(((g_md_h[PIC_TYPE2].rcv_data[1] >> 3) & 0b00000001) == 1){
		_launch_R_S1 = true;
	}else{
		_launch_R_S1 = false;
	}
	if(((g_md_h[PIC_TYPE2].rcv_data[1] >> 2) & 0b00000001) == 1){
		_launch_R_S2 = true;
	}
	if(((g_md_h[PIC_TYPE2].rcv_data[1] >> 5) & 0b00000001) == 1){
		_launch_L_S1 = true;
	}else{
		_launch_L_S1 = false;
	}
	if(((g_md_h[PIC_TYPE2].rcv_data[1] >> 4) & 0b00000001) == 1){
		_launch_L_S2 = true;
	}
	if(((g_md_h[PIC_TYPE1].rcv_data[1] >> 4) & 0b00000001) == 1){
		Pocket_In_Number = 1;
	}else if(((g_md_h[PIC_TYPE1].rcv_data[1] >> 5) & 0b00000001) == 1){
		Pocket_In_Number = 2;
	}else if(((g_md_h[PIC_TYPE1].rcv_data[1] >> 6) & 0b00000001) == 1){
		Pocket_In_Number = 3;
	}else if(((g_md_h[PIC_TYPE1].rcv_data[1] >> 7) & 0b00000001) == 1){
		Pocket_In_Number = 4;
	}else{
		Pocket_In_Number = 0;
	}
	if(((g_md_h[PIC_TYPE1].rcv_data[0] >> 0) & 0b00000001) == 1){
		_sens_pos_1 = true;
	}else{
		_sens_pos_1 = false;
	}
	if(((g_md_h[PIC_TYPE1].rcv_data[0] >> 1) & 0b00000001) == 1){
		_sens_pos_2 = true;
	}else{
		_sens_pos_2 = false;
	}
	if(((g_md_h[PIC_TYPE1].rcv_data[0] >> 2) & 0b00000001) == 1){
		_sens_pos_3 = true;
	}else{
		_sens_pos_3 = false;
	}
	if(((g_md_h[PIC_TYPE1].rcv_data[0] >> 3) & 0b00000001) == 1){
		_sens_pos_4 = true;
	}else{
		_sens_pos_4 = false;
	}
	if(((g_md_h[PIC_TYPE1].rcv_data[0] >> 4) & 0b00000001) == 1){
		_sens_pos_5 = true;
	}else{
		_sens_pos_5 = false;
	}
	if(((g_md_h[PIC_TYPE2].rcv_data[1]) & 0b00000001) == 1){
		rotate_direction = 1;
	}else if(((g_md_h[PIC_TYPE2].rcv_data[1] >> 1) & 0b00000001) == 1){
		rotate_direction = -1;
	}else{
		rotate_direction = 0;
	}

	if(_sens_pos_1){
		if(_sens_pos_1_enable){
			pocket = 0;
			if(_sens_pos_2 || _sens_pos_3 ||_sens_pos_4 ||_sens_pos_5){
				if(_sens_pos_2){
					pocket += 1;
				}
				if(_sens_pos_3){
					pocket += 2;
				}
				if(_sens_pos_4){
					pocket += 4;
				}
				if(_sens_pos_5){
					pocket += 8;
				}
				rotate_direction_byMech = getRotateDirectionByMech(Pocket_Number,pocket,rotate_direction_byMech);
				Pocket_Number = pocket;
				Pocket_Number_Detailed = pocket;
			}else{
				if(rotate_direction != 0){
					Pocket_Number_Detailed = Pocket_Number + (double)rotate_direction * 0.5;
				}else{
					Pocket_Number_Detailed = Pocket_Number + (double)rotate_direction_byMech * 0.5;
				}
				if(Pocket_Number_Detailed < 0.0){
					Pocket_Number_Detailed = 11.5;
				}else if(Pocket_Number_Detailed > 12.0){
					Pocket_Number_Detailed = 0.5;
				}
			}
		}
		_sens_pos_1_enable = false;
	}else{
		_sens_pos_1_enable = true;
	}


	if(enable_flag){
		for(i=0;i<DD_NUM_OF_MD;i++){
			g_md_h[i].mode = D_MMOD_IN_GAME;
		}


		if(_ball_InStop){
			if(Pocket_In_Number != 0){
				_ball_InStopped = true;
			}
			if(!_ball_InStopped){
				if(_Pos_control){

				}else{
					g_md_h[PIC_TYPE1].snd_data[0] = PC_control_rcv[3] & 0b01001111;
					g_md_h[PIC_TYPE1].snd_data[1] = PC_control_rcv[4];
				}
			}else{
				g_md_h[PIC_TYPE1].snd_data[0] &= 0b01000000;
				g_md_h[PIC_TYPE1].snd_data[1] = 0b00000000;
			}
		}else{
			_ball_InStopped = false;
			if(_Pos_control){
					
			}else{
				g_md_h[PIC_TYPE1].snd_data[0] = PC_control_rcv[3] & 0b01001111;
				g_md_h[PIC_TYPE1].snd_data[1] = PC_control_rcv[4];
			}
			
		}


		if(_is_manual){
			g_md_h[PIC_TYPE2].snd_data[0] = 0b00000000;
			g_md_h[PIC_TYPE2].snd_data[1] = PC_control_rcv[2] & 0b00000011;
		}
		if((PC_control_rcv[1] & 0b00000001) == 1){
			_launch_R_Ball = true;
		}
		if(((PC_control_rcv[1] >> 1) & 0b00000001) == 1){
			_launch_L_Ball = true;
		}
		if(_launch_R_Ball){
			if(_launch_R_S2 && _launch_R_S1){
				_launch_R_Ball = false;
				_launch_R_S2 = false;
				g_md_h[PIC_TYPE2].snd_data[1] &= 0b11111110;
			}else{
				g_md_h[PIC_TYPE2].snd_data[1] |= 0b00000001;
			}
		}
		if(_launch_L_Ball){
			if(_launch_L_S2 && _launch_L_S1){
				_launch_L_Ball = false;
				_launch_L_S2 = false;
				g_md_h[PIC_TYPE2].snd_data[1] &= 0b11111101;
			}else{
				g_md_h[PIC_TYPE2].snd_data[1] |= 0b00000010;
			}
		}
	}else{
		for(i=0;i<DD_NUM_OF_MD;i++){
			g_md_h[i].mode = D_MMOD_STANDBY;
		}
		g_md_h[PIC_TYPE1].snd_data[0] = 0b00000000;
		g_md_h[PIC_TYPE1].snd_data[1] = 0b00000000;
		g_md_h[PIC_TYPE2].snd_data[0] = 0b00000000;
		g_md_h[PIC_TYPE2].snd_data[1] = 0b00000000;
		_launch_R_Ball = false;
		_launch_L_Ball = false;
	}

	if( g_SY_system_counter % _MESSAGE_INTERVAL_MS < _INTERVAL_MS ){
		//MW_printf("%d",win_medal);
	}
	return EXIT_SUCCESS;
}

static
int getRotateDirectionByMech(int recentPocket, int nowPocket, int nowDirection){
	if(recentPocket == nowPocket) return nowDirection;
	int diff = nowPocket - recentPocket;
	if(diff == 11){
		return -1;
	}else if(diff == -11){
		return 1;
	}else{
		if(diff > 0){
			return 1;
		}else if(diff < 0){
			return -1;
		}
	}
}

static
bool JPrift_set(int side, int target_posi, bool reset){
	//target_posi = 0  lower
	int target_pic = side + 3;
	static now_position[2] = {0,0};
	static rift_count[2] = {0,0};
	int target = 0;
	const time_rift_posi1 = 100;

	if(reset){
		now_position[0] = 0;
		now_position[1] = 0;
		rift_count[0] = 0;
		rift_count[1] = 0;
		g_md_h[target_pic].snd_data[0] &= 0b11111100;
		g_md_h[target_pic].snd_data[1] = 0;
		return true; 
	}

	target = now_position[side] - target_posi;
	switch(target_posi){
	case 0:
		if(JPrift_get_status(side,1) == 0){
			JPrift_motor_setting(M_BACKWARD,100,side);
			return false;
		}else{
			now_position[side] = 0;
			JPrift_motor_setting(M_FREE,0,side);
			return true;
		}
		break;
	case 1:
	case 2:
	case 3:
		if(rift_count[side] >= abs(target)*time_rift_posi1){
			now_position[side] = target_posi;
			rift_count[side] = 0;
			JPrift_motor_setting(M_FREE,0,side);
			return true;
		}else{
			rift_count[side]++;
			if(target < 0){
				JPrift_motor_setting(M_FORWARD,100,side);
			}else{
				JPrift_motor_setting(M_BACKWARD,100,side);
			}
			return false;
		}
		break;
	case 4:
		if(JPrift_get_status(side,2) == 0){
			JPrift_motor_setting(M_FORWARD,100,side);
			return false;
		}else{
			now_position[side] = 4;
			JPrift_motor_setting(M_FREE,0,side);
			return true;
		}
		break;
	}

}

static
int JPrift_get_status(int side, int num){
	// num = 0  in
	// num = 1  rift_lower
	// num = 2  rift_upper
	int target_pic = side + 3;
	int return_data = 0;

	return_data = (g_md_h[target_pic].rcv_data[1] >> num) & 0b00000001;

	return return_data;
}

static
int outball_get_status(int side, int num){
	// num = 0  rail
	// num = 1  near pall
	// num = 2  bound
	int target_pic = side + 3;
	int return_data = 0;

	return_data = (g_md_h[target_pic].rcv_data[1] >> num+3) & 0b00000001;

	return return_data;
}

static
int outball_get_num(int side){
	int i;
	int return_num = 0;

	for(i=0; i<3; i++){
		if(outball_get_status(side,i) == 1){
			return_num++;
		}
	}
	return return_num;
}

static
int launchball_set(int side, bool reset){
	int target_pic = side + 3;
	static bool setting[2] = {false};

	if(reset){
		setting[side] = false;
		g_md_h[target_pic].snd_data[0] &= 0b11110111;
		return 0;
	}

	if(outball_get_status(side, 0) == 1){
		setting[side] = false;
		g_md_h[target_pic].snd_data[0] &= 0b11110111;
		return 0;
	}else if(outball_get_status(side, 1) == 1 && !setting[side]){
		setting[side] = true;
	}else if(outball_get_status(side, 1) == 0 && !setting[side]){
		g_md_h[target_pic].snd_data[0] &= 0b11110111;
		return -1;
	}
	if(setting[side]){
		g_md_h[target_pic].snd_data[0] |= 0b00001000;
		return 1;
	}
}

static 
BallLaunch_t ball_launch(int side, bool reset){
	BallLaunch_t return_state;
	int target_pic = side + 3;
	static bool launch[2] = {false};
	static int launch_count[2] = {0};

	if(reset){
		launchball_set(side,true);
		launch[side] = false;
		launch_count[side] = 0;
		g_md_h[target_pic].snd_data[0] &= 0b11111011;

		return BL_SETTING;
	}

	return_state = BL_SETTING;
	if(!launch[side]){
		if(launchball_set(side,false) == 0){
			launch[side] = true;
			return_state = BL_SETTING;
		}
		if(launchball_set(side,false) == -1){
			return_state = BL_NOBALL;
		}
	}else{
		launch_count[side]++;
		if(launch_count[side] >= 100){
			g_md_h[target_pic].snd_data[0] |= 0b00000100;
		}
		if(launch_count[side] >= 200){
			launch_count[side] = 200;
			g_md_h[target_pic].snd_data[0] &= 0b11111011;
			return_state = BL_LAUNCHED;
		}
	}

	return return_state;
}

static
bool JPC_get_status(int side, int *num){
	// num = 0  nothing
	// num = 1  JPJP
	int target_pic = side + 4;
	int i;

	for(i=1;i<=5;i++){
		if(((g_md_h[target_pic].rcv_data[1] >> i) & 0b00000001) == 1){
			*num = i;
			break;
		}
		*num = 0;
	}

	if((g_md_h[target_pic].rcv_data[1] & 0b00000001) == 1){
		return true;
	}else{
		return false;
	}
}

static 
int JPrift_motor_setting(Motor_direction_t direction, uint8_t duty, int side){
	int target_pic = side + 3;

	g_md_h[target_pic].snd_data[0] &= 0b11111100;
	g_md_h[target_pic].snd_data[0] |= direction;
	if(duty >= 100){
		g_md_h[target_pic].snd_data[1] = (uint8_t)(100.0 * 2.55);
	}else{
		g_md_h[target_pic].snd_data[1] = (uint8_t)((double)duty * 2.55);
	}
	return 0;
}

static 
int JPlottery_motor_setting(Motor_direction_t direction, uint8_t duty, int side){
	int target_pic = side + 4;

	g_md_h[target_pic].snd_data[0] &= 0b11111100;
	g_md_h[target_pic].snd_data[0] |= direction;
	if(duty >= 100){
		g_md_h[target_pic].snd_data[1] = (uint8_t)(100.0 * 2.55);
	}else{
		g_md_h[target_pic].snd_data[1] = (uint8_t)((double)duty * 2.55);
	}
	return 0;
}

static 
int center_motor_setting(Motor_direction_t direction, uint8_t duty){

	g_md_h[PIC_TYPE8].snd_data[0] &= 0b11111100;
	g_md_h[PIC_TYPE8].snd_data[0] |= direction;
	if(duty >= 100){
		g_md_h[PIC_TYPE8].snd_data[1] = (uint8_t)(100.0 * 2.55);
	}else{
		g_md_h[PIC_TYPE8].snd_data[1] = (uint8_t)((double)duty * 2.55);
	}
	return 0;
}

static
int outblock_setting(int side, int num, bool enable, bool stand, bool sit){
	int target_pic = side + 1;

	if(enable){
		if(num == 4){
			g_md_h[target_pic].snd_data[0] |= 0b00001111;
			g_md_h[target_pic].snd_data[1] &= 0b00000000;
		}else{
			g_md_h[target_pic].snd_data[0] |= 0b00000001 << num;
			g_md_h[target_pic].snd_data[1] &= 0b11111111 ^ (0b00000001 << num);
			g_md_h[target_pic].snd_data[1] &= 0b11111111 ^ (0b00000001 << (num+4));
		}
	}else{
		if(num == 4){
			g_md_h[target_pic].snd_data[0] &= 0b11110000;
		}else{
			g_md_h[target_pic].snd_data[0] &= 0b11111111 ^ (0b00000001 << num);
		}
	}
	if(stand){
		if(num == 4){
			g_md_h[target_pic].snd_data[1] |= 0b11110000;
			g_md_h[target_pic].snd_data[1] &= 0b11110000;
		}else{
			g_md_h[target_pic].snd_data[1] |= 0b00000001 << (num+4);
			g_md_h[target_pic].snd_data[1] &= 0b11111111 ^ (0b00000001 << num);
		}
	}else if(sit){
		if(num == 4){
			g_md_h[target_pic].snd_data[1] |= 0b00001111;
			g_md_h[target_pic].snd_data[1] &= 0b00001111;
		}else{
			g_md_h[target_pic].snd_data[1] |= 0b00000001 << num;
			g_md_h[target_pic].snd_data[1] &= 0b11111111 ^ (0b00000001 << (num+4));
		}
	}else{
		if(num == 4){
			g_md_h[target_pic].snd_data[1] &= 0b00000000;
		}else{
			g_md_h[target_pic].snd_data[1] &= 0b11111111 ^ (0b00000001 << num);
			g_md_h[target_pic].snd_data[1] &= 0b11111111 ^ (0b00000001 << (num+4));
		}
	}
}

static
int outblock_get_status(int side, int num){
	int target_pic = side + 1;
	int return_data = 0;

	return_data = (g_md_h[target_pic].rcv_data[1] >> num) & 0b00000001;

	return return_data;
}

static
int bumper_setting(int num, bool oneshot, bool enable){
	if(!oneshot){
		if(num == 4){
			if(enable){
				g_md_h[PIC_TYPE1].snd_data[1] |= 0b11110000;
			}else{
				g_md_h[PIC_TYPE1].snd_data[1] &= 0b00001111;
			}
		}else{
			if(enable){
				g_md_h[PIC_TYPE1].snd_data[1] |= (0x01 << (num+4));
			}else{
				g_md_h[PIC_TYPE1].snd_data[1] &= (0b11111111 ^ (0x01 << (num+4)));
			}
		}
	}else{
		if(num == 4){
			if(enable){
				g_md_h[PIC_TYPE1].snd_data[1] &= 0b00001111;
				g_md_h[PIC_TYPE1].snd_data[1] |= 0b00001111;
			}else{
				g_md_h[PIC_TYPE1].snd_data[1] &= 0b11110000;
			}
		}else{
			if(enable){
				g_md_h[PIC_TYPE1].snd_data[1] &= (0b11111111 ^ (0x01 << (num+4)));
				g_md_h[PIC_TYPE1].snd_data[1] |= (0x01 << num);
			}else{
				g_md_h[PIC_TYPE1].snd_data[1] &= (0b11111111 ^ (0x01 << num));
			}
		}
	}
}

static
int bumper_medal_get(bool recet){
	int recet_bit = 0;
	static int recent_read_bit = 0;
	static bool read_enable = true;
	int i;
	int count = 0;

	if(recet){
		read_enable = true;
		recent_read_bit = 0;
		return 0;
	}

	recet_bit = (g_md_h[PIC_TYPE1].rcv_data[1] >> 4) & 0b00000001;
	if(recet_bit == 0){
		g_md_h[PIC_TYPE1].snd_data[0] |= 0b00000001;
		g_md_h[PIC_TYPE1].snd_data[0] &= 0b11111101;
	}else if(recet_bit == 1){
		g_md_h[PIC_TYPE1].snd_data[0] |= 0b00000010;
		g_md_h[PIC_TYPE1].snd_data[0] &= 0b11111110;
	}
	if(recet_bit != recent_read_bit){
		read_enable = true;
	}
	if(read_enable){
		for(i=0;i<4;i++){
			if(((g_md_h[PIC_TYPE1].rcv_data[1] >> i) & 0b00000001) == 1) count++;
		}
		read_enable = false;
	}else{
		count = 0;
	}
	recent_read_bit = recet_bit;
	return count;
}