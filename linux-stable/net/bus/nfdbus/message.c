/*
 * message.c  Basic D-Bus message parsing
 *
 * Copyright (C) 2010-2012  Collabora Ltd
 * Authors:	Alban Crequy <alban.crequy@collabora.co.uk>
 * Copyright (C) 2002, 2003, 2004, 2005  Red Hat Inc.
 * Copyright (C) 2002, 2003  CodeFactory AB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 */

#include <linux/slab.h>

#include "message.h"

int dbus_message_type_from_string(const char *type_str)
{
	if (strcmp(type_str, "method_call") == 0)
		return DBUS_MESSAGE_TYPE_METHOD_CALL;
	if (strcmp(type_str, "method_return") == 0)
		return DBUS_MESSAGE_TYPE_METHOD_RETURN;
	else if (strcmp(type_str, "signal") == 0)
		return DBUS_MESSAGE_TYPE_SIGNAL;
	else if (strcmp(type_str, "error") == 0)
		return DBUS_MESSAGE_TYPE_ERROR;
	else
		return DBUS_MESSAGE_TYPE_INVALID;
}

int dbus_message_parse(unsigned char *message, size_t len,
		       struct dbus_message *dbus_message)
{
	unsigned char *cur;
	int array_header_len;

	dbus_message->message = message;

	if (len < 4 + 4 + 4 + 4 || message[1] == 0 || message[1] > 4)
		return -EINVAL;

	dbus_message->type = message[1];
	dbus_message->body_length = *((u32 *)(message + 4));
	cur = message + 12;
	array_header_len = *(u32 *)cur;
	dbus_message->len_offset = 12;
	cur += 4;
	while (cur < message + len
	       && cur < message + 12 + 4 + array_header_len) {
		int header_code;
		int signature_len;
		unsigned char *signature;
		int str_len;
		unsigned char *str;

		/* D-Bus alignment craziness */
		if ((cur - message) % 8 != 0)
			cur += 8 - (cur - message) % 8;

		header_code = *(char *)cur;
		cur++;
		signature_len = *(char *)cur;
		/* All header fields of the current D-Bus spec have a simple
		 * type, either o, s, g, or u */
		if (signature_len != 1)
			return -EINVAL;
		cur++;
		signature = cur;
		cur += signature_len + 1;
		if (signature[0] != 'o' &&
		    signature[0] != 's' &&
		    signature[0] != 'g' &&
		    signature[0] != 'u')
			return -EINVAL;

		if (signature[0] == 'u') {
			cur += 4;
			continue;
		}

		if (signature[0] != 'g') {
			str_len = *(u32 *)cur;
			cur += 4;
		} else {
			str_len = *(char *)cur;
			cur += 1;
		}

		str = cur;
		switch (header_code) {
		case 1:
			dbus_message->path = str;
			break;
		case 2:
			dbus_message->interface = str;
			break;
		case 3:
			dbus_message->member = str;
			break;
		case 6:
			dbus_message->destination = str;
			break;
		case 7:
			dbus_message->sender = str;
			break;
		case 8:
			dbus_message->body_signature = str;
			break;
		}
		cur += str_len + 1;
	}

	dbus_message->padding_end = (8 - (cur - message) % 8) % 8;

	/* Jump to body D-Bus alignment craziness */
	if ((cur - message) % 8 != 0)
		cur += 8 - (cur - message) % 8;
	dbus_message->new_header_offset = cur - message;

	if (dbus_message->new_header_offset
	    + dbus_message->body_length != len) {
		pr_warn("Message truncated? " \
			"Header %d + Body %d != Length %zd\n",
			dbus_message->new_header_offset,
			dbus_message->body_length, len);
		return -EINVAL;
	}

	if (dbus_message->body_signature &&
	    dbus_message->body_signature[0] == 's') {
		int str_len;
		str_len = *(u32 *)cur;
		cur += 4;
		dbus_message->arg0 = cur;
		cur += str_len + 1;
	}

	if ((cur - message) % 4 != 0)
		cur += 4 - (cur - message) % 4;

	if (dbus_message->body_signature &&
	    dbus_message->body_signature[0] == 's' &&
	    dbus_message->body_signature[1] == 's') {
		int str_len;
		str_len = *(u32 *)cur;
		cur += 4;
		dbus_message->arg1 = cur;
		cur += str_len + 1;
	}

	if ((cur - message) % 4 != 0)
		cur += 4 - (cur - message) % 4;

	if (dbus_message->body_signature &&
	    dbus_message->body_signature[0] == 's' &&
	    dbus_message->body_signature[1] == 's' &&
	    dbus_message->body_signature[2] == 's') {
		int str_len;
		str_len = *(u32 *)cur;
		cur += 4;
		dbus_message->arg2 = cur;
		cur += str_len + 1;
	}

	if ((cur - message) % 4 != 0)
		cur += 4 - (cur - message) % 4;

	if (dbus_message->type == DBUS_MESSAGE_TYPE_SIGNAL &&
	    dbus_message->sender && dbus_message->path &&
	    dbus_message->interface && dbus_message->member &&
	    dbus_message->arg0 &&
	    strcmp(dbus_message->sender, "org.freedesktop.DBus") == 0 &&
	    strcmp(dbus_message->interface, "org.freedesktop.DBus") == 0 &&
	    strcmp(dbus_message->path, "/org/freedesktop/DBus") == 0) {
		if (strcmp(dbus_message->member, "NameAcquired") == 0)
			dbus_message->name_acquired = dbus_message->arg0;
		else if (strcmp(dbus_message->member, "NameLost") == 0)
			dbus_message->name_lost = dbus_message->arg0;
	}

	return 0;
}
