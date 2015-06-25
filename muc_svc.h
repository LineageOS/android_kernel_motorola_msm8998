#ifndef MUC_SVC_H__
#define MUC_SVC_H__

#include "svc_msg.h"

extern void muc_svc_attach(struct greybus_host_device *hd);
extern void muc_svc_detach(struct greybus_host_device *hd);
#endif
