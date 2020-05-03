//
// Created by Jackson Hall on 4/27/2020.
//

#ifndef PART_2___GRAPHICS_ALTERNATE_OBJECT_4_D_H
#define PART_2___GRAPHICS_ALTERNATE_OBJECT_4_D_H

#include "Object.h"


class Object4D : public Object {
public:
    /** Constructors */
    Object4D();
//    Object4D(const std::vector<edge4d>& edges);
    Object4D(const std::vector<point4d>& vertices,
            const std::vector<edge4d>& edges);
    Object4D(std::vector<std::shared_ptr<point4d>> vertices,
            std::vector<std::shared_ptr<edge4d>> edges);

    /** Getters */
    const std::vector<std::shared_ptr<point4d>>& getVertices() const;
    const std::vector<std::shared_ptr<edge4d>>& getEdges() const;

    /** Other Methods */
    void extrude(const spatialVector& direction) override;
    void draw(const Camera3D& camera3d, const Camera4D& camera4d) const
    override;

protected:
    /** Fields */
    std::vector<std::shared_ptr<point4d>> vertices;

    // Pairs between the points in the object
    std::vector<std::shared_ptr<edge4d>> edges;
};


#endif //PART_2___GRAPHICS_ALTERNATE_OBJECT_4_D_H
