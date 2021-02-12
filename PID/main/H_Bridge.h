#ifndef _H_BRIDGE_H_
#define _H_BRIDGE_H_

#include <stdint.h>

typedef enum {H_FOWARD, H_BACKWARD, H_OFF} h_direction_t;

class HBRIDGE {
	
	public:	

		void H_Bridge_Init(int in_plus_, int in_minus_, int enb_pwm_);

		void H_Bridge_Set_Pwm(uint8_t pwm_value_);

		void H_Bridge_Set_Dir(h_direction_t dir);

	private:
	
		int in_plus;  
	
		int in_minus; 
		
		int enb_pwm;
		
		uint8_t pwm_value = 0;

};

#endif /* _H_BRIDGE_H_ */
