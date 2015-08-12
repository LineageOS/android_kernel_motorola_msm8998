#ifndef MUC_SVC_H__
#define MUC_SVC_H__

#include "mods_nw.h"
#include "greybus.h"

/* HACK for hard coded routes */
#define MODS_INTF_SVC 0
#define MODS_INTF_AP  1
#define MODS_INTF_MUC 3

extern void muc_svc_attach(struct greybus_host_device *hd);
extern void muc_svc_detach(struct greybus_host_device *hd);

extern struct mods_dl_device *mods_create_dl_device(struct mods_dl_driver *drv,
		struct device *parent, u8 intf_id);
extern void mods_remove_dl_device(struct mods_dl_device *mods_dev);
extern int mods_dl_dev_attached(struct mods_dl_device *mods_dev);
#endif
