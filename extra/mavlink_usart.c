
// 请求飞控发送数据
void mav_request_data(USART_TypeDef* huart)
{
  mavlink_message_t msg;
  uint8_t Sendbuf[MAVLINK_MAX_PACKET_LEN];
  int Buflen;
  mavlink_msg_request_data_stream_pack(100, 200, &msg, 1, MAV_COMP_ID_ALL, MAV_DATA_STREAM_ALL, 50, 1);
  Buflen = mavlink_msg_to_send_buffer(Sendbuf, &msg);
  BSP_USART_SendArray_LL(huart, Sendbuf, Buflen);

}


if(mavlink_parse_char(MAVLINK_COMM_0, data, &msg, &status))
    {
      //printf("ID: %d\r\n", msg.msgid);
      switch (msg.msgid)
      {
        // case MAVLINK_MSG_ID_ATTITUDE:
        //   pitch = mavlink_msg_attitude_get_pitch(&msg);
        //   yaw = mavlink_msg_attitude_get_yaw(&msg);
        //   roll = mavlink_msg_attitude_get_roll(&msg);					//   printf("pitch:%f, roll:%f\r\n", pitch, roll);
        //   break;
        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
          printf("ok_height\r\n");
          height = (float)mavlink_msg_distance_sensor_get_current_distance(&msg);
          printf("height:%f\r\n", height);
          break;
        // case MAVLINK_MSG_ID_VFR_HUD:
        //   printf("ok_height\r\n");
        //   height = mavlink_msg_vfr_hud_get_alt(&msg);
        //   printf("height:%f\r\n", height);
        //   break;
        case MAVLINK_MSG_ID_HIGHRES_IMU:
          printf("okk_height!\r\n");
          height = mavlink_msg_highres_imu_get_pressure_alt(&msg);
          printf("height:%f\r\n", height);
          break;
        case MAVLINK_MSG_ID_HIL_SENSOR:
          printf("okkk_height!\r\n");
          height = mavlink_msg_hil_sensor_get_pressure_alt(&msg);
          printf("height:%f\r\n", height);
          break;
        case MAVLINK_MSG_ID_HEARTBEAT:
          printf("ok\r\n");
          break;
        default:
          break;
      }
      LL_USART_DisableIT_RXNE(USART3);
      BSP_USART_StartIT_LL(USART3);
    }
    if(!write_fifo(&mav_fifo, data))
    {
      uint8_t fifo_data;
      while(read_fifo(&mav_fifo, &fifo_data))
      {
        printf("ok\r\n");
        if(mavlink_parse_char(MAVLINK_COMM_0, fifo_data, &msg, &status))
        {
          //mavlink_distance_sensor_t dis_msg;
          switch (msg.msgid)
          {
            case MAVLINK_MSG_ID_ATTITUDE:
               pitch = mavlink_msg_attitude_get_pitch(&msg);
              yaw = mavlink_msg_attitude_get_yaw(&msg);
              roll = mavlink_msg_attitude_get_roll(&msg);					      printf("pitch:%f, roll:%f\r\n", pitch, roll);
              break;
        // case MAVLINK_MSG_ID_DISTANCE_SENSOR:
        //   mavlink_msg_distance_sensor_decode(&msg, &dis_msg);
        //   height = (float)dis_msg.current_distance;
        //   printf("height:%f\r\n", height);
        //   break;
            case MAVLINK_MSG_ID_HEARTBEAT:
              printf("ok\r\n");
              break;
            default:
              break;
          }
        }
      }
      write_fifo(&mav_fifo, data);
      LL_USART_DisableIT_RXNE(USART3);
      BSP_USART_StartIT_LL(USART3);
    }