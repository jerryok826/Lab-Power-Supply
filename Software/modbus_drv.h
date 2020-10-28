#ifndef __MODBUS_DRV_H
#define __MODBUS_DRV_H

void init_registers (void);
int modbus_process (char *rx_buf, int mosbua_pkt_len, char *tx_buf);

#endif /* __MODBUS_H */
