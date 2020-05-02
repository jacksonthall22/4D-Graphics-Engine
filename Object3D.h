//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_OBJECT3D_H
#define PART_2___GRAPHICS_ALTERNATE_OBJECT3D_H

#include "Object.h"


class Object3D : protected Object {
public:
    /** Constructors */
    Object3D();
    Object3D(const std::vector<point3d>& vertices,
            const std::vector<edge3d>& edges);
    Object3D(std::vector<std::shared_ptr<point3d>> vertices,
        std::vector<std::shared_ptr<edge3d>> edges);

    /** Getters */
    const std::vector<std::shared_ptr<point3d>>& getVertices() const;
    const std::vector<std::shared_ptr<edge3d>>& getEdges() const;

    /** Other Methods */
    void extrude(const spatialVector& direction) override;
    void draw(const Camera3D& camera3d, const Camera4D& camera4d) const
    override;

protected:
    /** Fields */
    std::vector<std::shared_ptr<point3d>> vertices;

    // Pairs between the points in the object
    std::vector<std::shared_ptr<edge3d>> edges;

};


#endif //PART_2___GRAPHICS_ALTERNATE_OBJECT3D_H
