/*
 * Copyright 2017, Blender Foundation.
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
 * Contributor: IRIE Shinsuke
 */

#include "COM_DistanceTransformNode.h"
#include "COM_DistanceTransformOperation.h"
#include "COM_ExecutionSystem.h"

DistanceTransformNode::DistanceTransformNode(bNode *editorNode) : Node(editorNode)
{
	/* pass */
}

void DistanceTransformNode::convertToOperations(NodeConverter &converter, const CompositorContext &/*context*/) const
{
	DistanceTransformOperation *operation = new DistanceTransformOperation();
	bNode *editorNode = this->getbNode();
	operation->setThreshold(editorNode->custom3);
	operation->setInvert(editorNode->custom1);
	converter.addOperation(operation);

	converter.mapInputSocket(getInputSocket(0), operation->getInputSocket(0));
	converter.mapOutputSocket(getOutputSocket(0), operation->getOutputSocket(0));
}
