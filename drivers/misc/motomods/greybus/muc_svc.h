#ifndef MUC_SVC_H__
#define MUC_SVC_H__

#include "mods_nw.h"
#include "greybus.h"

/* SVC and AP are pre-defined interfaces */
#define MODS_INTF_SVC 0
#define MODS_INTF_AP  1
#define MODS_INTF_SIM  5

extern struct mods_dl_device *mods_create_dl_device(struct mods_dl_driver *drv,
		struct device *parent, u8 intf_id);
extern void mods_remove_dl_device(struct mods_dl_device *mods_dev);
extern int mods_dl_dev_attached(struct mods_dl_device *mods_dev);
extern void mods_dl_dev_detached(struct mods_dl_device *mods_dev);
extern void mods_dl_device_get(struct mods_dl_device *mods_dev);
extern void mods_dl_device_put(struct mods_dl_device *mods_dev);
extern void muc_svc_communication_reset(struct mods_dl_device *error_dev);
#endif
