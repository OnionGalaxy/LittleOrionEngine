#include "OLQuadTree.h"
#include <queue>

OLQuadTree::OLQuadTree()
{
}

OLQuadTree::~OLQuadTree()
{
	Clear();
}

void OLQuadTree::Create(AABB2D limits)
{
	Clear();
	root = new OLQuadTreeNode(limits);
	flattened_tree.push_back(root);
}

void OLQuadTree::Clear()
{
	for (auto & node : flattened_tree)
	{
		delete node;
	}
	flattened_tree.clear();
	root = nullptr;
}

void OLQuadTree::Insert(GameObject &game_object)
{
	if (root->box.Intersects(game_object.aabb.bounding_box2D))
	{
		OLQuadTreeNode *object_leaf = FindLeaf(game_object.aabb.bounding_box2D); //TODO: What happens if the object is inside more one node?
		assert(object_leaf != nullptr);
		if (object_leaf->objects.size() == bucket_size)
		{
			std::vector<OLQuadTreeNode*> generated_nodes;
			object_leaf->Split(generated_nodes);
			flattened_tree.insert(flattened_tree.end(), generated_nodes.begin(), generated_nodes.end());

			bool inserted = false;
			unsigned int i = 0;
			while (!inserted && i < 4)
			{
				if (generated_nodes[i]->box.Intersects(game_object.aabb.bounding_box2D))
				{
					generated_nodes[i]->InsertGameObject(&game_object);
					inserted = true;
				}
				++i;
			}
			assert(inserted);
		}
		else
		{
			object_leaf->InsertGameObject(&game_object);
		}
	}
	//TODO: What happens if object is outside the tree?

}

void OLQuadTree::CollectIntersect(std::vector<GameObject*> &game_objects, const ComponentCamera &camera)
{

}

OLQuadTreeNode* OLQuadTree::FindLeaf(const AABB2D &game_object_aabb) const
{
	for (auto &node : flattened_tree)
	{
		if (node->IsLeaf() && node->box.Intersects(game_object_aabb))
		{
			return node;
		}
	}

	return nullptr;
}