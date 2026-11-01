/****************************************************************************
 * apps/myprojects/modules/cdc_svc/cdc_service_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * p1ctl NSH command: execute one JSON request via CDC service.
 ****************************************************************************/
/****************************************************************************
 * 把 p1ctl 命令行收到的 JSON 交给 cdc_service.c。协议本身不在这个短文件里。
 ****************************************************************************/


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "../../include/cdc_service_api.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  static char resp[768];

  if (argc < 2)
    {
      printf("Usage: p1ctl '{\"cmd\":\"sys.ping\",\"id\":1}'\n");
      return 1;
    }

  memset(resp, 0, sizeof(resp));
  p1_cdc_handle_json(argv[1], resp, sizeof(resp));

  if (resp[0] != '\0')
    {
      printf("%s\n", resp);
    }
  else
    {
      printf("{\"ok\":false,\"code\":-1,\"msg\":\"empty\"}\n");
    }

  return 0;
}
