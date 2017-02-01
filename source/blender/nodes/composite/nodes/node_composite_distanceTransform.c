/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2017 Blender Foundation.
 * All rights reserved.
 *
 * The Original Code is: all of this file.
 *
 * Contributor(s): IRIE Shinsuke
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file blender/nodes/composite/nodes/node_composite_distanceTransform.c
 *  \ingroup cmpnodes
 */


#include "node_composite_util.h"

/* **************** Premul and Key Alpha Convert ******************** */

static bNodeSocketTemplate cmp_node_distance_transform_in[] = {
	{	SOCK_FLOAT, 1, N_("Mask"),			1.0f, 1.0f, 1.0f, 1.0f},
	{	-1, 0, ""	}
};
static bNodeSocketTemplate cmp_node_distance_transform_out[] = {
	{	SOCK_FLOAT, 0, N_("Distance")},
	{	SOCK_FLOAT, 0, N_("Vector X")},
	{	SOCK_FLOAT, 0, N_("Vector Y")},
	{	-1, 0, ""	}
};

static void node_composit_init_distance_transform(bNodeTree *UNUSED(ntree), bNode *node)
{
	node->custom1 = false;
	node->custom2 = false;
	node->custom3 = 0.5f;
}

void register_node_type_cmp_distance_transform(void)
{
	static bNodeType ntype;

	cmp_node_type_base(&ntype, CMP_NODE_DISTANCE_TRANSFORM, "Distance Transform", NODE_CLASS_CONVERTOR, 0);
	node_type_socket_templates(&ntype, cmp_node_distance_transform_in, cmp_node_distance_transform_out);
	node_type_init(&ntype, node_composit_init_distance_transform);

	nodeRegisterType(&ntype);
}
