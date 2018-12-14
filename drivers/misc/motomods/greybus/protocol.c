/*
 * Greybus protocol handling
 *
 * Copyright 2014-2015 Google Inc.
 * Copyright 2014-2015 Linaro Ltd.
 *
 * Released under the GPLv2 only.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "greybus.h"

/* Global list of registered protocols */
static DEFINE_SPINLOCK(gb_protocols_lock);
static LIST_HEAD(gb_protocols);

/* Caller must hold gb_protocols_lock */
static struct gb_protocol *gb_protocol_find(u8 id, u8 major, u8 minor)
{
	struct gb_protocol *protocol;

	list_for_each_entry(protocol, &gb_protocols, links) {
		if (protocol->id < id)
			continue;
		if (protocol->id > id)
			break;

		if (protocol->major > major)
			continue;
		if (protocol->major < major)
			break;

		if (protocol->minor > minor)
			continue;
		if (protocol->minor < minor)
			break;

		return protocol;
	}
	return NULL;
}

/* Caller must hold gb_protocols_lock */
static struct gb_protocol *gb_protocol_find_latest(u8 id, u8 major, bool match)
{
	struct gb_protocol *protocol;

	list_for_each_entry(protocol, &gb_protocols, links) {
		if (protocol->id < id)
			continue;
		if (protocol->id > id)
			break;

		if (!match)
			return protocol;

		if (protocol->major > major)
			continue;
		if (protocol->major < major)
			break;

		return protocol;
	}
	return NULL;
}

int __gb_protocol_register(struct gb_protocol *protocol, struct module *module)
{
	struct gb_protocol *existing;
	u8 id = protocol->id;
	u8 major = protocol->major;

	protocol->owner = module;

	/*
	 * The protocols list is sorted first by protocol id (low to
	 * high), then by major version (high to low), and finally
	 * by minor version (high to low).  Searching only by
	 * protocol id will produce the newest implemented version
	 * of the protocol.
	 */
	spin_lock_irq(&gb_protocols_lock);

	list_for_each_entry(existing, &gb_protocols, links) {
		if (existing->id < id)
			continue;
		if (existing->id > id)
			break;

		if (existing->major > major)
			continue;
		if (existing->major < major)
			break;

		/* A matching protocol has already been registered */
		spin_unlock_irq(&gb_protocols_lock);

		pr_err("Duplicate %s protocol major v%u\n",
			protocol->name, protocol->major);

		return -EEXIST;
	}

	/*
	 * We need to insert the protocol here, before the existing one
	 * (or before the head if we searched the whole list)
	 */
	list_add_tail(&protocol->links, &existing->links);
	spin_unlock_irq(&gb_protocols_lock);

	pr_info("Registered %s protocol v%u.%u.\n", protocol->name,
		protocol->major, protocol->minor);

	return 0;
}
EXPORT_SYMBOL_GPL(__gb_protocol_register);

/*
 * De-register a previously registered protocol.
 */
void gb_protocol_deregister(struct gb_protocol *protocol)
{
	if (!protocol)
		return;

	spin_lock_irq(&gb_protocols_lock);
	protocol = gb_protocol_find(protocol->id, protocol->major,
				    protocol->minor);
	if (WARN_ON(!protocol || protocol->count)) {
		spin_unlock_irq(&gb_protocols_lock);
		return;
	}

	list_del(&protocol->links);
	spin_unlock_irq(&gb_protocols_lock);

	pr_info("Deregistered %s protocol.\n", protocol->name);
}
EXPORT_SYMBOL_GPL(gb_protocol_deregister);

/* Returns the requested protocol if available, or a null pointer */
struct gb_protocol *gb_protocol_get(u8 id, u8 major, u8 minor)
{
	struct gb_protocol *protocol;
	u8 protocol_count;

	spin_lock_irq(&gb_protocols_lock);
	protocol = gb_protocol_find(id, major, minor);
	if (protocol) {
		if (!try_module_get(protocol->owner)) {
			protocol = NULL;
		} else {
			protocol_count = protocol->count;
			if (protocol_count != U8_MAX)
				protocol->count++;
		}
	}
	spin_unlock_irq(&gb_protocols_lock);

	if (protocol)
		WARN_ON(protocol_count == U8_MAX);

	return protocol;
}

/* Returns the latest protocol version supported, or null pointer */
struct gb_protocol *gb_protocol_get_latest(u8 id, u8 major, bool match)
{
	struct gb_protocol *protocol;
	u8 protocol_count;

	spin_lock_irq(&gb_protocols_lock);
	protocol = gb_protocol_find_latest(id, major, match);
	if (protocol) {
		if (!try_module_get(protocol->owner)) {
			protocol = NULL;
		} else {
			protocol_count = protocol->count;
			if (protocol_count != U8_MAX)
				protocol->count++;
		}
	}
	spin_unlock_irq(&gb_protocols_lock);

	if (protocol)
		WARN_ON(protocol_count == U8_MAX);

	return protocol;
}

int gb_protocol_get_version(struct gb_connection *connection)
{
	struct gb_protocol_version_request request;
	struct gb_protocol_version_response response;
	struct gb_protocol *protocol = connection->protocol;
	int retval;

	request.major = protocol->major;
	request.minor = protocol->minor;

	retval = gb_operation_sync(connection, GB_REQUEST_TYPE_PROTOCOL_VERSION,
				   &request, sizeof(request), &response,
				   sizeof(response));
	if (retval)
		return retval;

	if (response.major > connection->protocol->major) {
		dev_err(&connection->hd->dev,
			"%s: unsupported major version (%u > %u)\n",
			connection->name, response.major,
			connection->protocol->major);
		return -ENOTSUPP;
	}

	connection->module_major = response.major;
	connection->module_minor = response.minor;

	dev_dbg(&connection->hd->dev,
		"%s: %s (0x%02x) v%u.%u\n", connection->name,
		protocol->name, protocol->id, response.major, response.minor);

	return 0;
}
EXPORT_SYMBOL_GPL(gb_protocol_get_version);

int gb_protocol_version_negotiate(struct gb_connection *connection)
{
	struct gb_protocol *protocol = connection->protocol;
	struct gb_protocol *old_protocol;
	int ret;

	ret = gb_protocol_get_version(connection);

	/* If exact match, we're done */
	if (!ret && connection->module_major == protocol->major)
		return 0;

	if (ret && ret != -EPROTONOSUPPORT)
		return ret;

	dev_info(&connection->hd->dev,
		"%s: %s (0x%02x) v%u.%u mismatch v%u.%u\n",
		connection->name, protocol->name, protocol->id,
		protocol->major, protocol->minor,
		connection->module_major, connection->module_minor);

	/* Save off old protocol in case we find a new match */
	old_protocol = protocol;

	/* Try to find a matching major */
	protocol = gb_protocol_get_latest(protocol->id,
					connection->module_major,
					true);
	if (!protocol)
		return -EPROTONOSUPPORT;

	/* Drop reference to the old module */
	gb_protocol_put(old_protocol);

	connection->protocol = protocol;

	return gb_protocol_get_version(connection);
}
EXPORT_SYMBOL_GPL(gb_protocol_version_negotiate);

void gb_protocol_put(struct gb_protocol *protocol)
{
	u8 id;
	u8 major;
	u8 minor;

	id = protocol->id;
	major = protocol->major;
	minor = protocol->minor;

	spin_lock_irq(&gb_protocols_lock);
	protocol = gb_protocol_find(id, major, minor);
	if (WARN_ON(!protocol || !protocol->count))
		goto out;

	protocol->count--;
	module_put(protocol->owner);
out:
	spin_unlock_irq(&gb_protocols_lock);
}
