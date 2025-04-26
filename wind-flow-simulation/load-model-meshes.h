#pragma once
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "stb_image.h"
class Model {
private:
	Assimp::Importer importer;
	const aiScene* scene = nullptr;
	aiNode* root_node = nullptr;

	struct Mesh {
		unsigned int VAO, VBO1, VBO2, VBO3, VBO_stress, EBO;

		std::vector<glm::vec3> vert_positions;
		std::vector<glm::vec3> vert_normals;
		std::vector<glm::vec2> tex_coords;
		std::vector<unsigned int> vert_indices;
		unsigned int tex_handle;
	};

	struct Texture {
		unsigned int textureID;
		std::string image_name;
	};

public:
	// delete copy constructor
	Model(const Model&) = delete;

	// add move constructor
	Model(Model&& other) noexcept {
		// transfer from other to this
		mesh_list = std::move(other.mesh_list);
		num_meshes = other.num_meshes;
		other.num_meshes = 0;
	}

	// move assignment operator
	Model& operator=(Model&& other) noexcept {
		if (this != &other) {
			mesh_list = std::move(other.mesh_list);
			num_meshes = other.num_meshes;
			other.num_meshes = 0;
		}
		return *this;
	}

	unsigned int num_meshes;
	std::vector<Mesh> mesh_list;
	std::vector<Texture> texture_list;

	Model(const char* model_path) {
		scene = importer.ReadFile(model_path, aiProcess_JoinIdenticalVertices | aiProcess_Triangulate | aiProcess_FlipUVs);
		load_model();
	}

	void set_buffer_data(unsigned int index, const std::vector<float>& stress_values) {
		glGenVertexArrays(1, &mesh_list[index].VAO);
		glGenBuffers(1, &mesh_list[index].VBO1);
		glGenBuffers(1, &mesh_list[index].VBO2);
		glGenBuffers(1, &mesh_list[index].VBO3);
		glGenBuffers(1, &mesh_list[index].EBO);
		glGenBuffers(1, &mesh_list[index].VBO_stress);

		glBindVertexArray(mesh_list[index].VAO);

		// Vertex Positions
		glBindBuffer(GL_ARRAY_BUFFER, mesh_list[index].VBO1);
		glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * mesh_list[index].vert_positions.size(), &mesh_list[index].vert_positions[0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

		// Vertex Normals
		glBindBuffer(GL_ARRAY_BUFFER, mesh_list[index].VBO2);
		glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * mesh_list[index].vert_normals.size(), &mesh_list[index].vert_normals[0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);

		// Texture Coordinates
		glBindBuffer(GL_ARRAY_BUFFER, mesh_list[index].VBO3);
		glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * mesh_list[index].tex_coords.size(), &mesh_list[index].tex_coords[0], GL_STATIC_DRAW);
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

		// Indices for: glDrawElements()
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh_list[index].EBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * mesh_list[index].vert_indices.size(), &mesh_list[index].vert_indices[0], GL_STATIC_DRAW);

		glBindVertexArray(0);
	}


private:
	void load_model() {
		if (!scene || !scene->mRootNode || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE)
			std::cout << "Assimp importer.ReadFile (Error) -- " << importer.GetErrorString() << "\n";
		else {
			num_meshes = scene->mNumMeshes;
			mesh_list.resize(num_meshes);

			aiMesh* mesh{};
			int indices_offset = 0;

			// loop through the model meshes

			for (unsigned int i = 0; i < num_meshes; ++i) {
				mesh = scene->mMeshes[i]; // http://assimp.sourceforge.net/lib_html/structai_mesh.html				

				aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex]; // http://assimp.sourceforge.net/lib_html/structai_material.html			

				for (unsigned int tex_count = 0; tex_count < material->GetTextureCount(aiTextureType_DIFFUSE); ++tex_count) {
					aiString string;
					material->GetTexture(aiTextureType_DIFFUSE, tex_count, &string); // get name of the image file		

					// load mesh [i]'s texture if not already loaded
					int already_loaded = is_image_loaded(string.C_Str()); // returns -1 if texture not loaded, otherwise returns texture

					if (already_loaded == -1) { // image not loaded, attempt load
						bool load_success = false;
						unsigned int texture_handle = load_texture_image(string.C_Str(), load_success);

						if (load_success) { // image fails do nothing
							Texture texture;
							texture.image_name = string.C_Str();
							texture.textureID = texture_handle;

							texture_list.push_back(texture);
							mesh_list[i].tex_handle = texture_handle;
						}
					}
					else
						mesh_list[i].tex_handle = already_loaded;
				}
				// loop through all mesh [i]'s vertices
				for (unsigned int i2 = 0; i2 < mesh->mNumVertices; ++i2) {
					glm::vec3 position{};
					position.x = mesh->mVertices[i2].x;
					position.y = mesh->mVertices[i2].y;
					position.z = mesh->mVertices[i2].z;
					mesh_list[i].vert_positions.push_back(position);

					if (mesh->HasNormals()) {
						glm::vec3 normal{};
						normal.x = mesh->mNormals[i2].x;
						normal.y = mesh->mNormals[i2].y;
						normal.z = mesh->mNormals[i2].z;
						mesh_list[i].vert_normals.push_back(normal);
					}
					else
						mesh_list[i].vert_normals.push_back(glm::vec3(0.0f, 0.0f, 0.0f));

					if (mesh->HasTextureCoords(0)) {
						glm::vec2 tex_coords{};
						tex_coords.x = mesh->mTextureCoords[0][i2].x;
						tex_coords.y = mesh->mTextureCoords[0][i2].y;
						mesh_list[i].tex_coords.push_back(tex_coords);
					}
					else
						mesh_list[i].tex_coords.push_back(glm::vec2(0.0f, 0.0f));
				}
				// loop through all mesh [i]'s indices
				for (unsigned int i3 = 0; i3 < mesh->mNumFaces; ++i3)
					for (unsigned int i4 = 0; i4 < mesh->mFaces[i3].mNumIndices; ++i4)
						mesh_list[i].vert_indices.push_back(mesh->mFaces[i3].mIndices[i4] + indices_offset);

				std::vector<float> default_stress(mesh_list[i].vert_positions.size(), 0.0f); // placeholder stress values
				set_buffer_data(i, default_stress); // set up VAO, VBO and EBO.
			}
		}
	}

	int is_image_loaded(std::string file_name) {
		for (unsigned int i = 0; i < texture_list.size(); ++i)
			if (file_name.compare(texture_list[i].image_name) == 0)
				return texture_list[i].textureID;
		return -1;
	}

	unsigned int load_texture_image(std::string file_name, bool& load_complete) {
		std::size_t position = file_name.find_last_of("\\");
		file_name = "Images\\" + file_name.substr(position + 1);

		int width, height, num_components;
		unsigned char* image_data = stbi_load(file_name.c_str(), &width, &height, &num_components, 0);

		unsigned int textureID;
		glGenTextures(1, &textureID);

		if (image_data) {
			GLenum format{};

			if (num_components == 1)
				format = GL_RED;
			else if (num_components == 3)
				format = GL_RGB;
			else if (num_components == 4)
				format = GL_RGBA;

			glBindTexture(GL_TEXTURE_2D, textureID);
			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

			glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, image_data);
			glGenerateMipmap(GL_TEXTURE_2D);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_MIRRORED_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_MIRRORED_REPEAT);

			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

			load_complete = true;
			stbi_image_free(image_data);
			std::cout << "image loaded: " << file_name << "\n";
		}
		else {
			load_complete = false;
			stbi_image_free(image_data);
			std::cout << "image failed to load: " << file_name << "\n";
		}
		return textureID;
	}
};