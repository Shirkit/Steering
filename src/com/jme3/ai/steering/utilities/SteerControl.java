package com.jme3.ai.steering.utilities;

import com.jme3.asset.AssetManager;
import com.jme3.bullet.PhysicsSpace;
import java.util.List;

/**
 *
 * @author Shirkit
 */
public interface SteerControl {

    /**
     * Retrives all obstacals in the current scene.
     * 
     * @return a list of obstacals.
     */
    public List<Obstacle> getObstacals();

    /**
     * Retrieves all the obstacles nearby a target, given a radius.
     * 
     * @param radius the radius of how close the obstacals should be.
     * @param source the source position that we are looking from.
     * 
     * @return a list of the nearby obstacals.
     */
    public List<Obstacle> obstacalsNearby(AbstractVehicle source, float radius);

    /**
     * Retrieves all the neighbours nearby a target, given a radius.
     * 
     * @param radius the radius of how close the neighbours should be.
     * @param source the source position that we are looking from.
     * 
     * @return a list of the nearby neighbours.
     */
    public List<Obstacle> neighboursNearby(AbstractVehicle source, float radius);

    public AssetManager getAssetManager();

    public PhysicsSpace getPhysicsSpace();
}
