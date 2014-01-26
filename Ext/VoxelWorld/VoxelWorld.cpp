/*
	This file is part of the MinSG library.
	Copyright (C) 2014 Claudius J�hn <claudius@uni-paderborn.de>
	
	This library is subject to the terms of the Mozilla Public License, v. 2.0.
	You should have received a copy of the MPL along with this library; see the 
	file LICENSE. If not, you can obtain one at http://mozilla.org/MPL/2.0/.
*/
#ifdef MINSG_EXT_VOXEL_WORLD
#include "VoxelWorld.h"

#include <Rendering/Mesh/Mesh.h>
#include <Rendering/MeshUtils/MeshBuilder.h>
#include <Geometry/PointOctree.h>
#include <Geometry/Sphere.h>
#include <Geometry/VoxelStorage.h>
#include <Util/Graphics/Color.h>
#include <iostream>
#include <unordered_map>

#include <Util/Timer.h>
#include <map>


typedef Geometry::_Vec3<int32_t>	vec3i;
typedef Geometry::Vec3 				vec3f;
typedef uint32_t					value_t;


namespace std{
template <> struct hash<vec3i> {
	size_t operator()(const vec3i & v) const noexcept {	return static_cast<uint64_t>(v.x())*(129731+v.y())^v.z();	}
};
template <> struct hash<std::pair<vec3i,vec3i>> {
	size_t operator()(const std::pair<vec3i,vec3i> & p) const noexcept {	
		return hash<vec3i>()(p.first) - hash<vec3i>()(p.second);	}
};
}

namespace MinSG {

struct LightProbeData{
	struct LightProbeOctreePoint{
		const vec3f position; //redundant!
		const size_t lightProbeIndex;
		LightProbeOctreePoint(const vec3f & p,size_t idx) : position(p),lightProbeIndex(idx){}
		const vec3f& getPosition()const{	return position;	}
	};
	struct LightProbe{
		Util::Color4f color, lastPassColor;
		vec3f position,normal;
		std::vector<std::pair<const LightProbe*,float>> connectedProbes; //(probe,1/distanceSquared)
		LightProbe(const Util::Color4f& c) : color(c){}
		LightProbe(const Util::Color4f& c,const vec3f& pos) : color(c),position(pos){}
		LightProbe(const Util::Color4f& c,const vec3f& pos,const vec3f& n) : color(c),position(pos),normal(n){}
	};
	
	std::unordered_map<std::pair<vec3i,vec3i>,size_t> vertexLightProbeIndices; // pos,normal -> index
	std::vector<size_t> orderedVertexLightProbeIndices;
	Geometry::PointOctree<LightProbeOctreePoint> lightProbeOctree; // pos ->index
	std::vector<LightProbe> lightProbes;
		
	LightProbeData(uint32_t _wx, uint32_t _wy, uint32_t _wz) : 
			lightProbeOctree(Geometry::Box( vec3f(0,0,0),vec3f(_wx,_wy,_wz)),4,8){
	}
	
	Util::Color4f getVertexLightProbe(const vec3i& pos,const vec3i& normal)const{
		static const  Util::Color4f black;
		const auto& it = vertexLightProbeIndices.find(std::make_pair(pos,normal));
		return it==vertexLightProbeIndices.end() ? black : lightProbes.at( it->second ).color;
	}
	// create a lightProbe for pos,normal and return it, or return nullptr if it exists
	Util::Color4f* initVertexLightProbe(const vec3i& pos,const vec3i& normal){
		size_t index = lightProbes.size();
		auto result = vertexLightProbeIndices.emplace(std::make_pair(vec3i(pos),vec3i(normal)),index);
		if(result.second){
			lightProbes.emplace_back(Util::Color4f(),vec3f(pos),vec3f(normal));
			lightProbeOctree.insert(LightProbeOctreePoint(vec3f(pos),index));
			orderedVertexLightProbeIndices.push_back(index);
			return &lightProbes[index].color;
		}else{
			return nullptr;
		}
		
	}
	Util::Color4f& createFreeLightProbe(const vec3f& pos){
		const size_t index = lightProbes.size();
		lightProbes.emplace_back(Util::Color4f());
		lightProbeOctree.insert(LightProbeOctreePoint(pos,index));
		return lightProbes[index].color;
	}
};


struct VoxelGrid{
	const uint32_t wx,wy,wz,wxy;
	std::vector<uint32_t> voxels; // voxel, edgeflags, occlusion value
	
	VoxelGrid(uint32_t _wx, uint32_t _wy, uint32_t _wz) : 
			wx(_wx), wy(_wy), wz(_wz), wxy(wx*wy), voxels(wx*wy*wz){
	}
	
	void set(uint32_t x,uint32_t y,uint32_t z,value_t value)	{	voxels[ x+y*wx+z*wxy ] = value;	}

	
	inline value_t get(int32_t x,int32_t y,int32_t z)const{
		return (x<0||x>=static_cast<int32_t>(wx)||y<0||y>=static_cast<int32_t>(wy)||z<0||z>=static_cast<int32_t>(wz)) ? 0 : voxels[ x+y*wx+z*wxy ];
	}
	inline value_t get(const vec3i& v)const						{	return get(v.x(),v.y(),v.z());	}

	
	uint32_t getIndex(const vec3i& v)const						{	return v.x()+v.y()*wx+v.z()*wxy;	}
	vec3i clamp(const vec3i& v)const{
		return vec3i(
				std::max( std::min(v.x(),static_cast<int32_t>(wx)),0 ),
				std::max( std::min(v.y(),static_cast<int32_t>(wy)),0 ),
				std::max( std::min(v.z(),static_cast<int32_t>(wz)),0 )
		);
	}

	inline bool isTransparent(value_t value)const{
		return value==0;
	}

	/*! Cast a ray from @p source to @p target.
		If no block was hit, the distance is returned;
		Otherwise, the negative distance to the first intersection is returned.
		\todo support casting beyond the grid's boundaries.	
	*/
	float cast(const vec3f& source, const vec3f& target)const{
		vec3f dir = target-source;
		const float distance=dir.length();
		if(distance==0)
			return -1;
		dir/=distance;
		
		const float stepSize = 0.13f;
		const vec3f step(dir*stepSize);
		
		vec3f v = source + dir*0.01f;
		int32_t x=source.x()-10/*arbitrary invalid value*/, y=0, z=0;
		
		for(float currentRayLength = 0.01f; currentRayLength<distance-0.01f; currentRayLength+=stepSize){
			if(static_cast<int32_t>(v.x())!=x || static_cast<int32_t>(v.y())!=y || static_cast<int32_t>(v.z())!=z){
				x = static_cast<int32_t>(v.x());
				y = static_cast<int32_t>(v.y());
				z = static_cast<int32_t>(v.z());
				if(get(x,y,z)!=0 ){
					return -currentRayLength;
				}
			}
			v += step;
		}
		return distance;
		
	}
};


static void createVertex(Rendering::MeshUtils::MeshBuilder&mb, LightProbeData& lighting,int32_t x,int32_t y,int32_t z, value_t value,const vec3i& normal){

	Util::Color4f vertexColor;
	if(value>0){
		vertexColor = Util::Color4f(value,value,value,1.0);
	}
	const Util::Color4f light = lighting.getVertexLightProbe(vec3i(x,y,z),normal);
	
	mb.color( Util::Color4f( vertexColor.getR() * light.getR(),
							vertexColor.getG() * light.getG(),
							vertexColor.getB() * light.getB(), vertexColor.getA() ));
	mb.position(vec3f(x,y,z));
	mb.addVertex();
}


/*! Init a vertex light probe if it does not already exist.
	Applies global lighting to the probe.	*/
static void initVertexLightProbe(VoxelGrid& grid,LightProbeData& lighting,const vec3i& pos,const vec3i& normal){
	auto probe = lighting.initVertexLightProbe(pos,normal);
	if(probe){
		const vec3f pos2(pos);

		{// collect long range lighting 
			static const vec3f globalLight( 7.49,7.57,7.55);
			if( vec3f(normal).dot( globalLight-pos2 )>0){
				const float l = grid.cast( pos2,globalLight);
				if(l>0)
					*probe += Util::Color4f( 2.0f/l,0.01,0.01,0.0  );
			}
		}
		
		{// collect long range lighting 
			static const vec3f globalLight( 7.49,30.57,7.55);
			if( vec3f(normal).dot( globalLight-pos2 )>0){
				const float l = grid.cast( pos2,globalLight);
				if(l>0)
					*probe += Util::Color4f( 0.5,0.45,0.46,1.0  );
			}
		}
		
	
		*probe += Util::Color4f( 0.02,0.02,0.02,0.0  );
	}
}

//static void initEmitterLightProbe(VoxelGrid& grid,const vec3f& pos,value_t value){
//	const int x = pos.x();
//	const int y = pos.y();
//	const int z = pos.z();
//	if( (x%7)==0 && (y%7)==0 &&  (z%7)==0  ){
//		grid.createFreeLightProbe( pos ) = Util::Color4f( 0,20.1,0); // light emitter
//	}
//}
//initEmitterLightProbe

Util::Reference<Rendering::Mesh> VoxelWorld::generateMesh( const simpleVoxelStorage_t& voxelStorage, const Geometry::_Box<int32_t>& boundary){
	/*
	0. fill grid
	
	1. create probes
		for all blocks
			create surface probes with global lighting
			collect mid range light emitters
	
	2. mid range lighting
		for all mid range light emitters
			find surface probes and add mid range lighting
		
	3. create connections
		for all surface probes
			connect to reachable surface probes
		
	4. indirect lighting
		distribute reflected light
		
	5. create mesh
		for all blocks
			create mesh
	
	*/
	
	std::map<std::string,float> timings;
	Util::Timer t;
	

	
	VoxelGrid grid(boundary.getExtentX(),boundary.getExtentY(),boundary.getExtentZ());
	LightProbeData lighting(boundary.getExtentX(),boundary.getExtentY(),boundary.getExtentZ());

	// ---------------------------------------------------------------------------
	{ //	0. fill grid
		const auto data=voxelStorage.serialize(boundary);
		// fill uniform blocks
		for(const auto & area : data.first){
			const auto sidelength = std::get<1>(area);
			const auto min = grid.clamp( std::get<0>(area)-boundary.getMin() );
			const auto max = grid.clamp( std::get<0>(area)-boundary.getMin()+vec3i(sidelength,sidelength,sidelength) );
			const auto value = std::get<2>(area);
					
			for(int32_t z=min.z(); z<max.z(); ++z){
				for(int32_t y=min.y(); y<max.y(); ++y){
					for(int32_t x=min.x(); x<max.x(); ++x){
						grid.set(x,y,z,value);
					}
				}
			}
		}
		// fill leaf node blocks
		for(const auto & leafNode : data.second){
			const auto block = std::get<1>(leafNode);
			const uint32_t startIndex = grid.getIndex( std::get<0>(leafNode)- boundary.getMin() );

			for(uint32_t blockIndex=0; blockIndex<simpleVoxelStorage_t::blockSize; ++blockIndex){
				const value_t value = block[blockIndex];
				if(value!=0){
					const uint32_t voxelIndex = startIndex + blockIndex%simpleVoxelStorage_t::blockSideLength +
												grid.wx * ((blockIndex/simpleVoxelStorage_t::blockSideLength)%simpleVoxelStorage_t::blockSideLength) +
												grid.wxy * ((blockIndex/(simpleVoxelStorage_t::blockSideLength*simpleVoxelStorage_t::blockSideLength))%simpleVoxelStorage_t::blockSideLength);
					if(voxelIndex<grid.voxels.size())
						grid.voxels[voxelIndex] = value;
				}
			}
		}
		timings["10_fill grid"] = t.getMilliseconds();
		t.reset();
	}
	
	// ---------------------------------------------------------------------------
	//	1. create probes

	std::vector<LightProbeData::LightProbe> midRangeLightEmitters;
	
	// Traverse the grid in a zig-zag-manner to improve locality of orderedVertexLightProbeIndices
	for(uint32_t z2=0; z2<grid.wz; z2+=3){
	for(uint32_t y2=0; y2<grid.wy; y2+=3){
	for(uint32_t x2=0; x2<grid.wx; x2+=3){
	for(uint32_t z=z2; z<z2+3; ++z){
	for(uint32_t y=y2; y<y2+3; ++y){
	for(uint32_t x=x2; x<x2+3; ++x){
		const vec3i pos(x,y,z);
		const value_t value = grid.get(pos);
		if(value!=0)
			continue;
			
		// \todo allow emitting blocks!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		if( (x%7)==0 && (y%7)==1 &&  (z%7)==0  ){
			if(grid.get(x-1,y,z) !=0 || grid.get(x+1,y,z) !=0 || grid.get(x,y-1,z) !=0  || grid.get(x,y+1,z) !=0 
					 || grid.get(x,y,z-1) !=0  || grid.get(x,y,z+1) !=0  )
				midRangeLightEmitters.emplace_back( Util::Color4f( 0,2.1,0),vec3f(x+0.5f,y+0.5f,z+0.5f),vec3f(0,0,0) ); // light emitter
		}
		
		value_t value2;
		if( (value2=grid.get(x-1,y,z))!=0 ){
			static const vec3i normal(1,0,0);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y  ,z)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y+1,z)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y+1,z+1),normal);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y  ,z+1),normal);
//					initEmitterLightProbe(grid,vec3f(x+0.1f,y+0.5f  ,z+0.5f),value2);
		}
		if( (value2=grid.get(x+1,y,z))!=0 ){
			static const vec3i normal(-1,0,0);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y  ,z  )  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y  ,z+1)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y+1,z+1)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y+1,z  )  ,normal);
//					initEmitterLightProbe(grid,vec3f(x+0.9f,y+0.5f  ,z+0.5f),value2);
		}
		if( (value2=grid.get(x,y+1,z))!=0 ){
			static const vec3i normal(0,-1,0);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y+1,z  )  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y+1,z  )  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y+1,z+1)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y+1,z+1)  ,normal);
//					initEmitterLightProbe(grid,vec3f(x+0.5f,y+0.9f,z+0.5f),value2);
		}
		if( (value2=grid.get(x,y-1,z))!=0 ){
			static const vec3i normal(0,1,0);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y  ,z  )  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y  ,z+1)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y  ,z+1)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y  ,z  )  ,normal);
//					initEmitterLightProbe(grid,vec3f(x+0.5f,y+0.1f,z+0.5f),value2);
		}
		if( (value2=grid.get(x,y,z+1))!=0 ){
			static const vec3i normal(0,0,-1);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y  ,z+1)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y+1,z+1)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y+1,z+1)  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y  ,z+1)  ,normal);
//					initEmitterLightProbe(grid,vec3f(x+0.5f,y+0.5f,z+0.9f),value2);
		}
		if( (value2=grid.get(x,y,z-1))!=0 ){
			static const vec3i normal(0,0,1);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y  ,z  )  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y  ,z  )  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x+1,y+1,z  )  ,normal);
			initVertexLightProbe(grid, lighting, vec3i(x  ,y+1,z  )  ,normal);
//					initEmitterLightProbe(grid,vec3f(x+0.5f,y+0.5f,z+0.1f),value);
		}
	}}}}}}
			
	timings["20_init light probes"] = t.getMilliseconds();
	t.reset();
	
	// ---------------------------------------------------------------------------
	
	{ //	2. mid range lighting  (midRangeLightEmitters -> vertexProbes)
		std::deque<LightProbeData::LightProbeOctreePoint> localProbes;
		for(const auto & emitter : midRangeLightEmitters){
			const auto& pos = emitter.position;
			lighting.lightProbeOctree.collectPointsWithinBox(Geometry::Box(pos,12,12,12),localProbes);
			for(const auto& probePoint : localProbes){

				const auto dir = probePoint.position-pos;
				
				if( dir.lengthSquared()<16 ){

					auto& vertexProbe = lighting.lightProbes.at( probePoint.lightProbeIndex );
					if(	(vertexProbe.normal.isZero() || vertexProbe.normal.dot(dir)<0) ) { //&& vertexProbe.lastPassColor.getR()>0.01 
						const float dist = grid.cast(pos, probePoint.position);
						if(dist>0)
							vertexProbe.color += emitter.color * (1.0f/(1.0f+dist*dist) );
	//						probe.connectedProbes.push_back(&vertexProbe);
					}
				}
			}
		}
	}

	
	//--------------------------------------------------------------------------------------------------------
	// 3. create connections (vertexProbe <-> vertexProbe)
	{
		std::deque<LightProbeData::LightProbeOctreePoint> localProbes;
		int octreeQueries = 0;
		int collectedPoints = 0;
		int casts = 0;
		
		vec3f probesAtPos(-100000,0,0);

		for(const auto index : lighting.orderedVertexLightProbeIndices){
			auto& probe = lighting.lightProbes[ index ];
			const vec3f pos(probe.position);
			const vec3f normal(probe.normal);
			
			if(pos.distanceSquared(probesAtPos)>10){
				localProbes.clear();
		//		lighting.lightProbeOctree.collectPointsWithinSphere(Geometry::Sphere_f(pos,4),probes2);
	//			lighting.lightProbeOctree.collectPointsWithinBox(Geometry::Box(pos,8,8,8),localProbes);
				lighting.lightProbeOctree.collectPointsWithinBox(Geometry::Box(pos,10,10,10),localProbes);
				probesAtPos = pos;
				++octreeQueries;
				collectedPoints+=localProbes.size();
			}
	//
			for(const auto& probePoint : localProbes){
				if(probePoint.lightProbeIndex == index) // point itself
					continue;
				const auto dir = probePoint.position-pos;
				
				if( normal.dot(dir)>=0  &&
					dir.lengthSquared()<16 ){

					const auto& otherProbe = lighting.lightProbes.at( probePoint.lightProbeIndex );
					
					
					if(	(otherProbe.normal.isZero() || otherProbe.normal.dot(dir)>0) ) { //&& otherProbe.lastPassColor.getR()>0.01 
						const float dist = grid.cast(pos, probePoint.position);
						++casts;
						if(dist>0)
	//						probe += otherProbe.lastPassColor * (0.2f/(1.0f+dist*dist) );
							probe.connectedProbes.emplace_back(&otherProbe,(0.1f/(1.0f+dist*dist)));
					}
				}
			}
			casts += probe.connectedProbes.size();
		}
		
		timings["31_queries"] = octreeQueries;
		timings["32_collected points"] = collectedPoints;
		timings["33_casts"] = casts;
		timings["30_light distribution"] = t.getMilliseconds();
		t.reset();
	}
	
	
	//--------------------------------------------------------------------------------------------------------
	// 4. indirect lighting
	
	for(uint8_t i=0;i<5;++i){
		for(auto& p : lighting.lightProbes){
			p.lastPassColor = p.color;
		}
		for(auto& probe : lighting.lightProbes){
			for(const auto& link : probe.connectedProbes){
				probe.color += link.first->lastPassColor * link.second;
			}
		}
	}
	
	//--------------------------------------------------------------------------------------------------------

//	for(auto& probe : grid.lightProbes){
//			const auto &c = probe.color;
//			probe.color.set( c.getR()>0 ? std::log(c.getR()) : 0,c.getG()>0 ? std::log(c.getG()) : 0,c.getB()>0 ? std::log(c.getB()) : 0,1.0);
//	}
	
	const vec3f unit(1,1,1);
	Rendering::MeshUtils::MeshBuilder mb;

	mb.color( Util::Color4f(0.5,0.5,0.5) );
	for(uint32_t z=0; z<grid.wz; ++z){
		for(uint32_t y=0; y<grid.wy; ++y){
			for(uint32_t x=0; x<grid.wx; ++x){
				const uint32_t value = grid.get(x,y,z);
				if(value!=0)
					continue;
				uint32_t value2;
				if( (value2=grid.get(x-1,y,z))!=0 ){
					static const vec3i normal(1,0,0);
					const uint32_t idx = mb.getNextIndex();
					mb.normal( vec3f(normal) );
					createVertex(mb, lighting, x  ,y  ,z  ,value2, normal);
					createVertex(mb, lighting, x  ,y+1,z  ,value2, normal);
					createVertex(mb, lighting, x  ,y+1,z+1,value2, normal);
					createVertex(mb, lighting, x  ,y  ,z+1,value2, normal);
					mb.addQuad(idx,idx+1,idx+2,idx+3);
				}
				if( (value2=grid.get(x+1,y,z))!=0 ){
					static const vec3i normal(-1,0,0);
					const uint32_t idx = mb.getNextIndex();
					mb.normal( vec3f(normal) );
					createVertex(mb, lighting, x+1,y  ,z  ,value2, normal);
					createVertex(mb, lighting, x+1,y  ,z+1,value2, normal);
					createVertex(mb, lighting, x+1,y+1,z+1,value2, normal);
					createVertex(mb, lighting, x+1,y+1,z  ,value2, normal);
					mb.addQuad(idx,idx+1,idx+2,idx+3);
				}
				if( (value2=grid.get(x,y+1,z))!=0 ){
					static const vec3i normal(0,-1,0);
					const uint32_t idx = mb.getNextIndex();
					mb.normal( vec3f(normal) );
					createVertex(mb, lighting, x  ,y+1,z  ,value2, normal);
					createVertex(mb, lighting, x+1,y+1,z  ,value2, normal);
					createVertex(mb, lighting, x+1,y+1,z+1,value2, normal);
					createVertex(mb, lighting, x  ,y+1,z+1,value2, normal);
					mb.addQuad(idx,idx+1,idx+2,idx+3);
				}
				if( (value2=grid.get(x,y-1,z))!=0 ){
					static const vec3i normal(0,1,0);
					const uint32_t idx = mb.getNextIndex();
					mb.normal( vec3f(normal) );
					createVertex(mb, lighting, x  ,y  ,z  ,value2, normal);
					createVertex(mb, lighting, x  ,y  ,z+1,value2, normal);
					createVertex(mb, lighting, x+1,y  ,z+1,value2, normal);
					createVertex(mb, lighting, x+1,y  ,z  ,value2, normal);
					mb.addQuad(idx,idx+1,idx+2,idx+3);
				}
				if( (value2=grid.get(x,y,z+1))!=0 ){
					static const vec3i normal(0,0,-1);
					const uint32_t idx = mb.getNextIndex();
					mb.normal( vec3f(normal) );
					createVertex(mb, lighting, x  ,y  ,z+1,value2, normal);
					createVertex(mb, lighting, x  ,y+1,z+1,value2, normal);
					createVertex(mb, lighting, x+1,y+1,z+1,value2, normal);
					createVertex(mb, lighting, x+1,y  ,z+1,value2, normal);
					mb.addQuad(idx,idx+1,idx+2,idx+3);
				}
				if( (value2=grid.get(x,y,z-1))!=0 ){
					static const vec3i normal(0,0,1);
					const uint32_t idx = mb.getNextIndex();
					mb.normal( vec3f(normal) );
					createVertex(mb, lighting, x  ,y  ,z  ,value2, normal);
					createVertex(mb, lighting, x+1,y  ,z  ,value2, normal);
					createVertex(mb, lighting, x+1,y+1,z  ,value2, normal);
					createVertex(mb, lighting, x  ,y+1,z  ,value2, normal);
					mb.addQuad(idx,idx+1,idx+2,idx+3);
				}
			}
		}
	}
	timings["40_build mesh"] = t.getMilliseconds();
	t.reset();

	for(auto& it:timings){
		std::cout << it.first<<"\t"<<it.second<<std::endl;
	}
	
	return mb.buildMesh();

}


}
#endif /* MINSG_EXT_VOXEL_WORLD */
