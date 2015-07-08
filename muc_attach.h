#ifndef _MUC_ATTACH_H__
#define _MUC_ATTACH_H__

#include <linux/notifier.h>

int register_muc_attach_notifier(struct notifier_block *nb);
int unregister_muc_attach_notifier(struct notifier_block *nb);

#endif /* _MUC_ATTACH_H__ */
