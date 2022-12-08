

#include <nv/mesh.h>

#include <iostream>
#include <fstream>
#include <vector>

namespace nv
{

    bool Mesh::save(const std::string &filename) const
    {
        if (filename.empty() || vertices.empty())
            return false;

        std::ofstream ply_file;
        ply_file.open(filename.c_str(), std::ios::binary);
        if (!ply_file.is_open())
            return false;

        // write ply file header
        bool has_colors = !colors.empty();
        int num_pts = static_cast<int>(vertices.size());
        int num_faces = static_cast<int>(face_vertices.size());
        ply_file << "ply" << std::endl;
        ply_file << "format binary_little_endian 1.0" << std::endl;
        ply_file << "element vertex " << num_pts << std::endl;
        ply_file << "property float x" << std::endl;
        ply_file << "property float y" << std::endl;
        ply_file << "property float z" << std::endl;
        if (has_colors)
        {
            ply_file << "property uchar red" << std::endl;
            ply_file << "property uchar green" << std::endl;
            ply_file << "property uchar blue" << std::endl;
        }
        ply_file << "element face " << num_faces << std::endl;
        ply_file << "property list uchar int vertex_indices" << std::endl;
        ply_file << "end_header" << std::endl;

        // write ply data (binary format)

        // write vertices
        for (size_t i = 0; i < vertices.size(); ++i)
        {
            // write vertex
            Vec3f v = vertices[i].cast<float>();
            ply_file.write((const char*)&v[0], sizeof(float) * 3);

            if (has_colors)
            {
                // write color
                ply_file.write((const char*)&colors[i][0], sizeof(unsigned char) * 3);
            }
        }

        // write faces
        unsigned char num_indices_per_face = 3;
        for (size_t i = 0; i < face_vertices.size(); ++i)
        {
            // face indices
            ply_file.write((const char*)&num_indices_per_face, sizeof(unsigned char));
            Eigen::Matrix<int, 3, 1> face_ind = face_vertices[i].cast<int>();
            ply_file.write((const char*)&face_ind[0], num_indices_per_face*sizeof(int));
        }

        ply_file.close();

        return true;
    }

} 
