#ifndef _CORERESOURCES_H_
#define _CORERESOURCES_H_

#include <unordered_map>

enum class CoreResource
{
	UNKNOWN = 0,
	CUBE = 1,
	SPHERE = 2,
	TORUS = 3,
	CYLINDER = 4,
	QUAD = 5,

	DEFAULT_MATERIAL = 6,

	DEFAULT_SKYBOX = 7,
	SKYBOX_TEXTURE_RIGHT = 8,
	SKYBOX_TEXTURE_LEFT = 9,
	SKYBOX_TEXTURE_UP = 10,
	SKYBOX_TEXTURE_DOWN = 11,
	SKYBOX_TEXTURE_FRONT = 12,
	SKYBOX_TEXTURE_BACK = 13,

	BILLBOARD_CAMERA_TEXTURE = 14,
	BILLBOARD_LIGHT_TEXTURE = 15
};

static std::unordered_map<std::string, CoreResource> core_resources_pathes({
	{ "/Resources/Meshes/Cube.mesh", CoreResource::CUBE },
	{ "/Resources/Meshes/Sphere.mesh", CoreResource::SPHERE },
	{ "/Resources/Meshes/Torus.mesh", CoreResource::TORUS },
	{ "/Resources/Meshes/Cylinder.mesh", CoreResource::CYLINDER },
	{ "/Resources/Meshes/Quad.mesh", CoreResource::QUAD },

	{ "/Resources/Materials/default.mat", CoreResource::DEFAULT_MATERIAL },

	{ "/Resources/Skyboxes/ame_nebula/default.skybox", CoreResource::DEFAULT_SKYBOX },
	{ "/Resources/Skyboxes/ame_nebula/purplenebula_rt.tga", CoreResource::SKYBOX_TEXTURE_RIGHT },
	{ "/Resources/Skyboxes/ame_nebula/purplenebula_lf.tga", CoreResource::SKYBOX_TEXTURE_LEFT },
	{ "/Resources/Skyboxes/ame_nebula/purplenebula_up.tga", CoreResource::SKYBOX_TEXTURE_UP },
	{ "/Resources/Skyboxes/ame_nebula/purplenebula_dn.tga", CoreResource::SKYBOX_TEXTURE_DOWN },
	{ "/Resources/Skyboxes/ame_nebula/purplenebula_ft.tga", CoreResource::SKYBOX_TEXTURE_FRONT },
	{ "/Resources/Skyboxes/ame_nebula/purplenebula_bk.tga", CoreResource::SKYBOX_TEXTURE_BACK },

	{ "/Resources/Textures/video-solid.png", CoreResource::BILLBOARD_CAMERA_TEXTURE },
	{ "/Resources/Textures/lightbulb-solid.png", CoreResource::BILLBOARD_LIGHT_TEXTURE },

});

#endif // !_CORERESOURCES_H_