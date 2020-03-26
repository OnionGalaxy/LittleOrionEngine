#include "Material.h"

#include "Helper/Config.h"
#include "Main/Application.h"
#include "Module/ModuleTexture.h"
#include "Module/ModuleResourceManager.h"

#include "ResourceManagement/Metafile/Metafile.h"

Material::Material(Metafile* resource_metafile) :
	Resource(resource_metafile)
{
	textures.resize(MAX_MATERIAL_TEXTURE_TYPES);
}

void Material::Save(Config& config) const
{
	for (size_t i = 0; i < textures.size(); i++)
	{
		if (textures[i] != nullptr)
		{
			MaterialTextureType type = static_cast<MaterialTextureType>(i);

			switch (type)
			{
			case MaterialTextureType::DIFFUSE:
				config.AddUInt(textures[i]->GetUUID(), "Diffuse");
				break;

			case MaterialTextureType::SPECULAR:
				config.AddUInt(textures[i]->GetUUID(), "Specular");
				break;

			case MaterialTextureType::OCCLUSION:
				config.AddUInt(textures[i]->GetUUID(), "Occlusion");
				break;

			case MaterialTextureType::EMISSIVE:
				config.AddUInt(textures[i]->GetUUID(), "Emissive");
				break;

			default:
				break;
			}
		}
	}

	config.AddBool(show_checkerboard_texture, "Checkboard");
	config.AddString(shader_program, "ShaderProgram");

	//k
	config.AddFloat(k_ambient, "kAmbient");
	config.AddFloat(k_specular, "kSpecular");
	config.AddFloat(k_diffuse, "kDiffuse");
	config.AddFloat(shininess, "shininess");

	//colors
	config.AddColor(float4(diffuse_color[0], diffuse_color[1], diffuse_color[2], diffuse_color[3]), "difusseColor");
	config.AddColor(float4(emissive_color[0], emissive_color[1], emissive_color[2], 1.0f), "emissiveColor");
	config.AddColor(float4(specular_color[0], specular_color[1], specular_color[2], 1.0f), "specularColor");
}

void Material::Load(const Config& config)
{
	uint32_t diffuse_uuid = config.GetUInt("Diffuse", 0);
	if (diffuse_uuid != 0)
	{
		std::shared_ptr<Texture> texture_resource = std::static_pointer_cast<Texture>(App->resources->Load(diffuse_uuid));
		SetMaterialTexture(Material::MaterialTextureType::DIFFUSE, texture_resource);
	}

	uint32_t specular_uuid = config.GetUInt("Specular", 0);
	if (specular_uuid != 0)
	{
		std::shared_ptr<Texture> texture_resource = std::static_pointer_cast<Texture>(App->resources->Load(specular_uuid));
		SetMaterialTexture(Material::MaterialTextureType::SPECULAR, texture_resource);
	}

	uint32_t occlusion_uuid = config.GetUInt("Occlusion", 0);
	if (occlusion_uuid != 0)
	{
		std::shared_ptr<Texture> texture_resource = std::static_pointer_cast<Texture>(App->resources->Load(occlusion_uuid));
		SetMaterialTexture(Material::MaterialTextureType::OCCLUSION, texture_resource);
	}

	uint32_t emissive_uuid = config.GetUInt("Emissive", 0);
	if (occlusion_uuid != 0)
	{
		std::shared_ptr<Texture> texture_resource = std::static_pointer_cast<Texture>(App->resources->Load(emissive_uuid));
		SetMaterialTexture(Material::MaterialTextureType::EMISSIVE, texture_resource);
	}

	show_checkerboard_texture = config.GetBool("Checkboard", true);
	config.GetString("ShaderProgram", shader_program, "Blinn phong");

	//k
	k_ambient = config.GetFloat("kAmbient", 1.0f);
	k_specular = config.GetFloat("kSpecular", 1.0f);
	k_diffuse = config.GetFloat("kDiffuse", 1.0f);
	shininess = config.GetFloat("shininess", 1.0f);

	//colors
	float4 diffuse;
	float4 emissive;
	float4 specular;

	config.GetColor("difusseColor", diffuse, float4(1.f, 1.f, 1.f, 1.f));
	config.GetColor("emissiveColor", emissive, float4(0.0f, 0.0f, 0.0f, 1.0f));
	config.GetColor("specularColor", specular, float4(0.0f, 0.0f, 0.0f, 1.0f));

	diffuse_color[0] = diffuse.x;
	diffuse_color[1] = diffuse.y;
	diffuse_color[2] = diffuse.z;
	diffuse_color[3] = diffuse.w;

	emissive_color[0] = emissive.x;
	emissive_color[1] = emissive.y;
	emissive_color[2] = emissive.z;
	emissive_color[3] = emissive.w;

	specular_color[0] = specular.x;
	specular_color[1] = specular.y;
	specular_color[2] = specular.z;
	specular_color[3] = specular.w;
}

void Material::RemoveMaterialTexture(MaterialTextureType type)
{
	textures[type] = nullptr;
}

void Material::SetMaterialTexture(MaterialTextureType type, const std::shared_ptr<Texture>& new_texture)
{
	textures[type] = new_texture;
}

const std::shared_ptr<Texture>& Material::GetMaterialTexture(MaterialTextureType type) const
{
	return textures[type];
}