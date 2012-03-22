/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jme3.ai.steering.utilities;

import java.util.ArrayList;
import jme3tools.navmesh.Path;

/**
 *
 * @author Shirkit
 */
public class Flock<T extends AbstractVehicle> extends ArrayList<T> {
    
    public Path path;
    public int lastPath = 0;
}
