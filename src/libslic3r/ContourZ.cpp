#include "Layer.hpp"

namespace Slic3r {

static void contour_extrusion_entity(Layer *layer, const sla::IndexedMesh &mesh, ExtrusionEntity *extr);

static bool contour_extrusion_path(Layer *layer, const sla::IndexedMesh &mesh, ExtrusionPath &path) {
	coordf_t mesh_z = layer->print_z + mesh.ground_level();
	coordf_t min_z = 0.05;

	const Points3 &points = path.polyline.points;
	double resolution_mm = 0.1;

	coordf_t height = layer->height;
	std::cout << "LAYER ID " << layer->id() << std::endl;
	std::cout << "PRINT Z " << layer->print_z << std::endl;
	std::cout << "LAYER HEIGHT " << height << std::endl;
	std::cout << "EXTRUSION HEIGHT " << path.height << std::endl;
	Pointf3s contoured_points;
	bool was_contoured = false;

	for (Points3::const_iterator it = points.begin(); it != points.end()-1; ++it) {
		Vec2d p1d(unscale_(it->x()), unscale_(it->y()));
		Vec2d p2d(unscale_((it+1)->x()), unscale_((it+1)->y()));
		Linef line(p1d, p2d);

		double length_mm = line.length();
		// std::cout << "LENGTH (mm) " << length_mm << "; P1 " << p1d << "; P2 " << p2d << std::endl;

		int num_segments = int(std::ceil(length_mm / resolution_mm));
		Vec2d delta = line.vector();

		for (int i = 0; i < num_segments+1; i++) {
			Vec2d p = p1d + delta*i/num_segments;

			coordf_t x = p.x();
			coordf_t y = p.y();

			// std::cout << "P x " << x << "; y " << y << "; z " << mesh_z << std::endl;

			sla::IndexedMesh::hit_result hit_up = mesh.query_ray_hit({x, y, mesh_z}, {0.0, 0.0, 1.0});
			sla::IndexedMesh::hit_result hit_down = mesh.query_ray_hit({x, y, mesh_z}, {0.0, 0.0, -1.0});

			// std::cout << "HIT UP " << hit_up.distance() << std::endl;
			// std::cout << "HIT DOWN " << hit_down.distance() << std::endl;

			double up = hit_up.distance();
			double down = hit_down.distance();
			double d = 0;

			if (up < down && up <= min_z) {
				d = std::min(min_z, up);
			} else if (down <= up && down <= height - min_z) {
				d = -std::min(height - min_z, down);
			}

			if (std::abs(d) > EPSILON) {
				was_contoured = true;
			}

			Vec3d new_point = {p.x(), p.y(), d};

			if (contoured_points.size() > 2) {
				double dist = Linef3::distance_to_infinite_squared(
					contoured_points[contoured_points.size() - 2], 
					contoured_points[contoured_points.size() - 1], 
					new_point);
				if (dist < EPSILON) {
					contoured_points[contoured_points.size() - 1] = new_point;
					continue;
				}
			}

			contoured_points.push_back(new_point);
		}
	}

	std::cout << "WAS CONTOURED " << was_contoured << std::endl;

	if (!was_contoured) {
		return false;
	}

	Polyline3 polyline;
	for (const Vec3d &point : contoured_points) {
		polyline.append(Point3(scale_(point.x()), scale_(point.y()), scale_(point.z())));
	}

	path.polyline = std::move(polyline);
	path.z_contoured = true;
	return true;
}

static void contour_extrusion_loop(Layer *layer, const sla::IndexedMesh &mesh, ExtrusionLoop &loop) 
{
	for (ExtrusionPath &path : loop.paths) {
		contour_extrusion_path(layer, mesh, path);
	}
}

static void contour_extrusion_entitiy_collection(Layer *layer, const sla::IndexedMesh &mesh, ExtrusionEntityCollection &collection) {
	for (ExtrusionEntity *entity : collection.entities) {
		contour_extrusion_entity(layer, mesh, entity);
	}
}

static void contour_extrusion_entity(Layer *layer, const sla::IndexedMesh &mesh, ExtrusionEntity *extr) {
	const ExtrusionPathSloped *sloped = dynamic_cast<const ExtrusionPathSloped*>(extr);
	if (sloped != nullptr) {
		throw RuntimeError("ExtrusionPathSloped not implemented");
		return;
	}

	ExtrusionPath *path = dynamic_cast<ExtrusionPath*>(extr);
	if (path != nullptr) {
		contour_extrusion_path(layer, mesh, *path);
		return;
	}

	ExtrusionLoop *loop = dynamic_cast<ExtrusionLoop*>(extr);
	if (loop != nullptr) {
		contour_extrusion_loop(layer, mesh, *loop);
		return;
	}

	const ExtrusionLoopSloped *loop_sloped = dynamic_cast<const ExtrusionLoopSloped*>(extr);
	if (loop_sloped != nullptr) {
		throw RuntimeError("ExtrusionLoopSloped not implemented");
		return;
	}

	ExtrusionEntityCollection *collection = dynamic_cast<ExtrusionEntityCollection*>(extr);
	if (collection != nullptr) {
		contour_extrusion_entitiy_collection(layer, mesh, *collection);
		return;
	}

	throw RuntimeError("ContourZ: ExtrusionEntity type not implemented");
	return;
}

static void handle_extrusion_collection(Layer *layer, const sla::IndexedMesh &mesh, ExtrusionEntityCollection &collection, ExtrusionRole role) {
	for (size_t i = 0; i < collection.entities.size(); i++) {
		ExtrusionEntity *extr = collection.entities[i];
		if (extr->role() != role) {
			continue;
		}

		contour_extrusion_entity(layer, mesh, extr);
	}
}

void Layer::make_contour_z(const sla::IndexedMesh &mesh)
{
	coordf_t z = this->print_z;
	std::cout << "layer print z: " << z << std::endl;

	for (LayerRegion *region : this->regions()) {
		for (size_t i = 0; i < region->fills.entities.size(); i++) {
			handle_extrusion_collection(this, mesh, region->fills, ExtrusionRole::erTopSolidInfill);
			handle_extrusion_collection(this, mesh, region->perimeters, ExtrusionRole::erExternalPerimeter);
		}
	}
}
} // namespace Slic3r