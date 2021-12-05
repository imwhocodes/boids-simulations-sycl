#pragma once
#include <Eigen/Dense>
#include <vector>
#include "GLWindow.hpp"

#include <memory>

float_t randFloat() {
    return float_t(rand()) / float_t(RAND_MAX);
}

float_t randFloat(const float_t max) {
    return (float_t(rand()) / float_t(RAND_MAX)) * max;
}

float_t randFloat(const float_t min, const float_t max) {
    return min + randFloat(max - min);
}



class Boid 
    :
public std::enable_shared_from_this<Boid>{

    public:

        using Coord_t = Eigen::Vector2f;
        using CoordList_t = Eigen::Matrix2Xf;
        using DistanceList_t = Eigen::VectorXf;
        using MaskList_t = Eigen::Array<bool, 1, Eigen::Dynamic>;

        using Ptr_t = std::shared_ptr<Boid>;
        using List_t = std::vector<Boid>;
        using PtrList_t = std::vector<Ptr_t>;

        static Coord_t HALF_SIMULATION_SIZE;

        static constexpr float_t BOID_SIZE  =  0.005;

        static float_t PERCEPTION_RADIUS;
        static float_t SEPARATION_RADIUS;
        static float_t ALIGNMENT_RADIUS;
        static constexpr float_t BBOX_RADIUS =        0.25;


        static float_t PERCEPTION_WEIGHT;
        static float_t SEPARATION_WEIGHT;
        static float_t ALIGNMENT_WEIGHT;
        static constexpr float_t BBOX_WEIGHT =        1;


        static constexpr float_t MAX_SPEED = 0.015;
        static constexpr float_t ACCELERATION = 0.003;


        static constexpr size_t ABS_MIN_BOID_QTY = 10;
        static constexpr size_t ABS_MAX_BOID_QTY = 5000;



    protected:

        const Coord_t position;
        const Coord_t velocity;

        static void assertPosition(const Coord_t & pos){
            assert(
                (
                    ( pos.array() >= -Boid::HALF_SIMULATION_SIZE.array() )
                        &&
                    ( pos.array() <=  Boid::HALF_SIMULATION_SIZE.array() )
                ).all()
            );
        }

        static void assertVelocity(const Coord_t & vel){
            assert(vel.norm() <= (Boid::MAX_SPEED + 0.01));
        }
    
    public:

        Boid(const float_t x, const float_t y)
            :
        Boid{Coord_t{x, y}, Coord_t::Zero()}
        {}

        Boid(const Coord_t & pos, const Coord_t & vel)
            :
        position{pos},
        velocity{vel}
        {
            Boid::assertPosition(pos);
            Boid::assertVelocity(vel);
        }

        const Coord_t & getPosition() const {
            return this->position;
        }


        Boid applyForce(const Coord_t & force) const {

            return this->applyAcceleration(force * Boid::ACCELERATION);
        }

        Boid applyAcceleration(const Coord_t & acceleration) const {

            return Boid{ 
                            Boid::ClampPosition( this->position + velocity ),
                            Boid::ClampVelocity( this->velocity + acceleration )
                        };
        }

        Boid applyNoForce() const {
            return this->applyAcceleration(Coord_t::Zero());
        }


        float_t distance(const Boid & other) const {
            return (this->position - other.position).norm();
        }

        void drawSelf() const {
            glColor3f(1.0, 1.0, 1.0);
            DrawFilledCircle(
                                this->position.x(),
                                this->position.y(),
                                Boid::BOID_SIZE
            );
        }


        Coord_t getForceFromBBox() const { 

            // const float_t radius = Eigen::Vector3f{
            //                                             Boid::PERCEPTION_RADIUS,
            //                                             Boid::SEPARATION_RADIUS,
            //                                             Boid::ALIGNMENT_RADIUS
            //                                     }.maxCoeff();

            return - ( Coord_t::Ones() - ( ( Boid::HALF_SIMULATION_SIZE - this->position.cwiseAbs() ).cwiseMin( Boid::BBOX_RADIUS ) / Boid::BBOX_RADIUS ) ).cwiseProduct(this->position.cwiseSign());

        }


        static Boid Random(){
            return { 
                        Boid::Coord_t::Random().cwiseProduct((Boid::HALF_SIMULATION_SIZE - Boid::Coord_t::Constant(Boid::BBOX_RADIUS))),
                        Boid::ClampVelocity( Coord_t::Random() * Boid::MAX_SPEED )
            };
        }

        static Coord_t ClampPosition(const Coord_t & position){
            return position.cwiseMax(-Boid::HALF_SIMULATION_SIZE).cwiseMin(Boid::HALF_SIMULATION_SIZE);
        }

        static Coord_t ClampVelocity(const Coord_t & velocity){

            const float_t abs_vel = velocity.norm();

            if(abs_vel <= Boid::MAX_SPEED){
                return velocity;
            }

            else{
                return velocity * (Boid::MAX_SPEED / abs_vel);
            }


            
        }


        static Boid::PtrList_t RandomPtrList(const size_t n){

            Boid::PtrList_t res;

            for (size_t i = 0; i < n; i++){

                res.push_back(std::make_shared<Boid>(Boid::Random()));
            }
            
            return res;

        }

        static Boid::PtrList_t UpdateBoidQuantity(const Boid::PtrList_t & input, const size_t qty){

            const size_t in_size = input.size();

            if(in_size < qty){

                Boid::PtrList_t res = input;

                const size_t to_add = qty - in_size;

                for(size_t i = 0; i < to_add; i++ ){
                    res.push_back(std::make_shared<Boid>(Boid::Random()));
                }

                return res;
            }

            else if(in_size > qty){

                Boid::PtrList_t res;
                res.reserve(qty);

                const size_t to_remove = in_size - qty;


                const size_t skip_every = ( (in_size + (to_remove-1) ) / to_remove );

                for(size_t i = 0; (i < in_size) && (res.size() < qty); i++ ){ // 

                    if( i % skip_every ){
                        res.push_back(input[i]);
                    }
                }

                assert(res.size() == qty);

                return res;

            }

            else{
                return input;
            }

            
        }

        static Boid::CoordList_t GeneratePositionList(const Boid::PtrList_t & input){
            Boid::CoordList_t res;

            res.resize(Eigen::NoChange, input.size());

            for(size_t i = 0; i < input.size(); i++){
                res.col(i) = input[i]->position;
            }

            return res;

        }


        static Boid::CoordList_t GenerateVelocityList(const Boid::PtrList_t & input){
            Boid::CoordList_t res;

            res.resize(Eigen::NoChange, input.size());

            for(size_t i = 0; i < input.size(); i++){
                res.col(i) = input[i]->velocity;
            }

            return res;

        }

        // static Boid::Coord_t GenerateAvgWithMask(const Boid::CoordList_t & values, const Boid::MaskList_t & mask){

        //     return values(Eigen::all, mask).rowwise().sum() / mask.count();

        // }

        static Boid::Coord_t IfNaNGetZero( const Boid::Coord_t & v ){
            if(v.array().isNaN().any()){
                return Boid::Coord_t::Zero();
            }
            else{
                return v;
            }
        }


        static Boid::DistanceList_t GenerateDistanceList(const Boid::CoordList_t & input, const Eigen::Index idx ){

            Boid::DistanceList_t dist = (input.colwise() - input.col(idx)).colwise().norm();

            dist(idx) = INFINITY;

            assert(dist.rows() == input.cols());
            
            return dist;

        }


        static auto ComputeMaskByDistance(const Boid::DistanceList_t & distances, const float_t max_distance){
            return (distances.array() <= max_distance).transpose();
        }

        static auto ComputeMaskByDistanceAndAngle(const Boid::DistanceList_t & distances, const float_t max_distance, const float_t max_radius){
            return (distances.array() <= max_distance).transpose();
        }


        static Boid::Coord_t ComputeAvgInRadius(const Boid::DistanceList_t & distances, const Boid::CoordList_t & values, const float_t max_distance){
            
            const auto mask = (distances.array() <= max_distance).transpose();

            const auto mask_sized = mask.replicate< Boid::Coord_t::RowsAtCompileTime, 1 >();

            const auto masked = mask_sized.select(values, 0);

            return  masked.rowwise().sum() / mask.count();

        }


        static Boid::PtrList_t MainComputeNewGeneration(const Boid::PtrList_t & input){

            Boid::PtrList_t res(input.size());

            const Boid::CoordList_t pos_list = Boid::GeneratePositionList(input);
            const Boid::CoordList_t vel_list = Boid::GenerateVelocityList(input);

            #pragma omp parallel for schedule(static) num_threads(16)
            for(size_t i = 0; i < input.size(); i++ ){

                const Boid & b = *(input[i]);

                const Boid::DistanceList_t dist_list = Boid::GenerateDistanceList(pos_list, i);


                const Boid::Coord_t force_of_bbox             = b.getForceFromBBox();



                const Boid::Coord_t center_of_group_perception =   Boid::ComputeAvgInRadius(dist_list, pos_list, Boid::PERCEPTION_RADIUS);
                const Boid::Coord_t force_of_group_perception =     (center_of_group_perception - b.position) / Boid::PERCEPTION_RADIUS;

                
                
                const Boid::Coord_t center_of_group_separation = Boid::ComputeAvgInRadius(dist_list, pos_list, Boid::SEPARATION_RADIUS);
                const Boid::Coord_t aplha_to_group_separation =  (center_of_group_separation - b.position) / Boid::SEPARATION_RADIUS;
                const Boid::Coord_t force_of_group_separation = - (Coord_t::Ones() - aplha_to_group_separation.cwiseAbs()).array() * aplha_to_group_separation.cwiseSign().array();


    
                const Boid::Coord_t velocity_of_group_alignment =   Boid::ComputeAvgInRadius(dist_list, vel_list, Boid::ALIGNMENT_RADIUS);
                const Boid::Coord_t force_of_group_velocity  =        (velocity_of_group_alignment - b.velocity) / (Boid::MAX_SPEED * 2);


                res[i] = std::make_shared<Boid>( 
                                                    b.applyForce( 
                                                                    (
                                                                        ( force_of_bbox * Boid::BBOX_WEIGHT )
                                                                            +
                                                                        ( Boid::IfNaNGetZero(force_of_group_perception) * Boid::PERCEPTION_WEIGHT )
                                                                            + 
                                                                        ( Boid::IfNaNGetZero(force_of_group_separation) * Boid::SEPARATION_WEIGHT )
                                                                            + 
                                                                        ( Boid::IfNaNGetZero(force_of_group_velocity)   * Boid::ALIGNMENT_WEIGHT )
                                                                    )
                                                                    //     /
                                                                    
                                                                    // (
                                                                    //     Boid::BBOX_WEIGHT
                                                                    //         +
                                                                    //     Boid::PERCEPTION_WEIGHT
                                                                    //         +
                                                                    //     Boid::SEPARATION_WEIGHT
                                                                    //         +
                                                                    //     Boid::ALIGNMENT_WEIGHT
                                                                    // )
                                                                ) 
                            );
               
            }

            return res;
        }


        static void SetSimulationSize(const Coord_t & size){
            Boid::HALF_SIMULATION_SIZE = size / 2;

            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            const Boid::Coord_t min = -Boid::HALF_SIMULATION_SIZE;
            const Boid::Coord_t max =  Boid::HALF_SIMULATION_SIZE;

            glOrtho(min.x(), max.x(), max.y(), min.y(), -1, 1);

            glMatrixMode(GL_MODELVIEW);
        }


        static Boid::PtrList_t SetSimulationSize(const Coord_t & size, const Boid::PtrList_t & input){
            
            const Coord_t old_half_size = Boid::HALF_SIMULATION_SIZE;
            
            Boid::SetSimulationSize(size);

            if( ( old_half_size.array() > Boid::HALF_SIMULATION_SIZE.array() ).any() ){

                Boid::PtrList_t res;

                for(const Boid::Ptr_t & b : input){
                    res.push_back( std::make_shared<Boid>( 
                                                            Boid::ClampPosition(b->position),
                                                            b->velocity
                                                        )
                    );
                }

                return res;

            }

            else{
                return input;
            }  
            
        }


        static void SetPerceptionRadius(const float_t r){
            assert(r >= 0 && r <= 1);
            Boid::PERCEPTION_RADIUS =  r;
        }

        static void SetSeparationRadius(const float_t r){
            assert(r >= 0 && r <= 1);
            Boid::SEPARATION_RADIUS = r;
        }

        static void SetAlignmentRadius(const float_t r){
            assert(r >= 0 && r <= 1);
            Boid::ALIGNMENT_RADIUS =  r;
        }


        static void SetPerceptionWeight(const float_t w){
            assert(r >= 0 && r <= 1);
            Boid::PERCEPTION_WEIGHT =  w;
        }

        static void SetSeparationWeight(const float_t w){
            assert(r >= 0 && r <= 1);
            Boid::SEPARATION_WEIGHT = w;
        }

        static void SetAlignmentWeight(const float_t w){
            assert(r >= 0 && r <= 1);
            Boid::ALIGNMENT_WEIGHT =  w;
        }



};

Boid::Coord_t Boid::HALF_SIMULATION_SIZE = Boid::Coord_t::Constant(0.5);

float_t Boid::PERCEPTION_RADIUS =  0.1;
float_t Boid::SEPARATION_RADIUS =  Boid::PERCEPTION_RADIUS / 4;
float_t Boid::ALIGNMENT_RADIUS =   Boid::PERCEPTION_RADIUS * 3;

float_t Boid::PERCEPTION_WEIGHT =  0.2;
float_t Boid::SEPARATION_WEIGHT =  0.75;
float_t Boid::ALIGNMENT_WEIGHT =   0.3;