        if (!(mcp23017_inputs & 1 << 6)) {
          // Need to read both status bytes to clear change int line 
          at42qt1070_det_status ();
          key_status_last = key_status;
          key_status = at42qt1070_key_status ();
          std_printf ("key_status: %d, key_status_last: %d\n", key_status, key_status_last);
          if (key_status == 0) {
            release_time = (unsigned int) xTaskGetTickCount ();
            press_time_diff = release_time - press_time;
            press_long = 0;
            if (press_time_diff > 1000) { // one second hold time
              press_long = 1;
            }
            std_printf ("Key release: 0x%02X, %lu, d_time: %lu, type: %lu\n", key_status, release_time,
                        press_time_diff, press_long);
          }
          else {
            press_time = (unsigned int) xTaskGetTickCount ();
            std_printf ("Key press:   0x%02X, %d\n", key_status,
                        (unsigned int) xTaskGetTickCount ());
          }
          if (key_status == 1) {        // mem1_voltage_set = voltage_set
            mem1_button_CB ();
//             start_time = systicks();
//             release_time = systicks();
#if 0
            release_time = release_time - start_time;
            if (release_time > 100) {
              std_printf ("long release\n");
//                mem1_voltage_set = voltage_set;
            }
            else {
              std_printf ("short release\n");
            }
            mem1_button_CB ();
#endif
          }

          if (key_status == 2) {
            mem2_button_CB ();
          }
          if (key_status == 4) {
            output_button_CB (0);
          }
        }
