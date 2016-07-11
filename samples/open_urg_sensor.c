/*!
  \brief ��sensor��Ӧ�Ľӿ� ���ӵ�URG sensor
  \author Satofumi KAMIMURA

  $Id: open_urg_sensor.c,v c5747add6615 2015/05/07 03:18:34 alexandr $
*/

#include "open_urg_sensor.h"
#include "urg_utils.h"
#include "urg_detect_os.h"
#include <string.h>
#include <stdio.h>


int open_urg_sensor(urg_t *urg, int argc, char *argv[])
{
	/* ������Ҫ���ӵĶ˿� */
    const char *device = "/dev/ttyACM0";
	/* �˿����ӷ�ʽ: ���� */
    urg_connection_type_t connection_type = URG_SERIAL;
	/* ������115200 */
    long baudrate_or_port = 115200;
    const char *ip_address = "192.168.0.10";
    int i;

    // �����豸
    if (urg_open(urg, connection_type, device, baudrate_or_port) < 0) {
		/* �˿ڴ�ʧ�� */
        printf("urg_open: %s, %ld: %s\n",
            device, baudrate_or_port, urg_error(urg));
        return -1;
    }
	DEBUG("Open sensor port successful!");

    return 0;
}
