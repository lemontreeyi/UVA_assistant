if((tar_x - current_location[0]) * (tar_x - current_location[0]) + (tar_y - current_location[1]) * (tar_y - current_location[1]) < 0.20*0.20)
    {

        next_location[0] = target_location[0];
        next_location[1] = target_location[1];
        return true;
    }
    else
    {
        float tan = (tar_y - current_location[1]) / (tar_x - current_location[0]);
        float sin = sqrt(tan * tan / (1 + tan * tan));
        float cos = sqrt(1 / (1 + tan * tan));
				//printf("tan:%f sin:%f cos:%f flag_x:%f flag_y:%f\r\n", tan, sin, cos, flag_x, flag_y);
        //转化到cm为单位
        //next_y
        next_location[1] = (int)((current_location[1] + 0.35 * flag_y * sin) * 100);
        //next_x
        next_location[0] = (int)((current_location[0] + 0.35 * flag_x * cos) * 100);
				//printf("%f %d\r\n", (current_location[1] + 0.35 * flag_y * sin) * 100, (int)((current_location[1] + 0.35 * flag_y * sin) * 100));
        return false;
    }