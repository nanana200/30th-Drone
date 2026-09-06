#ifndef INC_UAV_LINK_H_
#define INC_UAV_LINK_H_

#include "gnss.h"

#include <stdint.h>

HAL_StatusTypeDef uav_link_init(void);
void uav_link_update_gnss(const gnss_pvt_t *pvt);
void uav_link_process(void);

#endif /* INC_UAV_LINK_H_ */
