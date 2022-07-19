#include "MAV_Altitude_Decode.h"
#include "mavlink_types.h"
#include "stdio.h"
int Mav_Altitude_Decoder(uint16_t rxlen, mavlink_message_t msg, uint8_t *buf, float* height)
{
	mavlink_status_t status;
	mavlink_distance_sensor_t packet;
	uint16_t i=0;
	for(i=0;i<rxlen;i++)
	{
		if(mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
		{	
			switch(msg.msgid)
			{
				case MAVLINK_MSG_ID_DISTANCE_SENSOR:
					mavlink_msg_distance_sensor_decode(&msg,&packet);
					//printf("height received!\r\n");
					*height = packet.current_distance;
					return 1;
			}		
		}
	
	}
	return 0;
}
