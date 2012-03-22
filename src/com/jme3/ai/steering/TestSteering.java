package com.jme3.ai.steering;

import com.jme3.ai.steering.control.FlockerControl;
import com.jme3.ai.steering.control.PathFollowerControl;
import com.jme3.ai.steering.control.WanderControl;
import com.jme3.ai.steering.utilities.AbstractVehicle;
import com.jme3.ai.steering.utilities.Flock;
import com.jme3.ai.steering.utilities.Obstacle;
import com.jme3.ai.steering.utilities.SteerControl;
import com.jme3.app.SimpleApplication;
import com.jme3.asset.AssetManager;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.collision.CollisionResults;
import com.jme3.font.BitmapText;
import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Sphere;
import com.jme3.system.AppSettings;
import com.jme3.terrain.geomipmap.TerrainQuad;
import java.util.ArrayList;
import java.util.List;
import jme3tools.navmesh.NavMesh;
import jme3tools.navmesh.Path;

public class TestSteering extends SimpleApplication implements SteerControl, ActionListener {

    private Node vehicleNode;
    private Node obstacleNode;
    private Node friendNode;
    private BulletAppState bulletAppState;
    private TerrainQuad terrain;
    private NavMesh navmesh;
    public static TestSteering instance;

    public static void main(String[] args) {
        TestSteering app = new TestSteering();
        instance = app;
        app.setSettings(new AppSettings(true));
        app.settings.setHeight(720);
        app.settings.setWidth(1280);
        //app.settings.setVSync(true);
        app.setShowSettings(true);
        app.start();
    }

    @Override
    public void start() {
        super.start();
    }

    private void createTerrain() {
        Node cena = (Node) assetManager.loadModel("Scenes/cena.j3o");

        terrain = (TerrainQuad) cena.getChild("terrain-cena");
        Geometry navm = (Geometry) cena.getChild("NavMesh");

        Material m = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        m.getAdditionalRenderState().setWireframe(true);
        m.setColor("Color", ColorRGBA.Green);

        Geometry g = new Geometry("lixo", navm.getMesh());
        g.setMaterial(m);
        g.setCullHint(Spatial.CullHint.Never);

        navmesh = new NavMesh();
        navmesh.loadFromMesh(navm.getMesh());

        rootNode.attachChild(cena);
        rootNode.attachChild(g);
    }

    private Spatial createSphereAtPoint(Vector3f point, ColorRGBA color) {
        Geometry g = new Geometry(point.toString(), new Sphere(16, 16, 0.5f));
        g.setCullHint(Spatial.CullHint.Never);
        Material m = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        m.setColor("Color", color);
        g.setMaterial(m);
        g.setLocalTranslation(point);
        rootNode.attachChild(g);
        return g;
    }
    ArrayList<Spatial> array = new ArrayList<Spatial>();

    private Path navmesh(Vector3f init, Vector3f end) {
        for (Spatial s : array) {
            s.removeFromParent();
        }

        Path path = new Path();
        boolean buildNavigationPath = navmesh.buildNavigationPath(path, navmesh.findClosestCell(init), init, navmesh.findClosestCell(end), end, 0.4f);

        if (buildNavigationPath) {
            for (Path.Waypoint p : path.getWaypoints()) {
                Vector3f v = p.getPosition();

                array.add(createSphereAtPoint(v, ColorRGBA.Pink));
            }
            array.add(createSphereAtPoint(path.getEnd().getPosition(), ColorRGBA.Cyan));

            return path;
        }

        return null;
    }

    private void setupBasicKeys() {
        inputManager.addMapping("ResetVehicles", new KeyTrigger(KeyInput.KEY_R));
        inputManager.addListener(this, "ResetVehicles");

        inputManager.addMapping("1", new KeyTrigger(KeyInput.KEY_1));
        inputManager.addListener(this, "1");

        inputManager.addMapping("2", new KeyTrigger(KeyInput.KEY_2));
        inputManager.addListener(this, "2");

        inputManager.addMapping("3", new KeyTrigger(KeyInput.KEY_3));
        inputManager.addListener(this, "3");
    }

    private void setupCamera() {
        getCamera().setLocation(new Vector3f(0, 20, 0));
        getCamera().lookAt(Vector3f.ZERO, Vector3f.UNIT_X);
        getFlyByCamera().setMoveSpeed(50);
    }

    private void createVehicles() {
        vehicleNode = new Node("Vehicles");
        obstacleNode = new Node("Obstacles");
        friendNode = new Node("Friends");

        int amount = 200;
        
        // create obstacles
        for (int i = 0; i < amount; i++) {
            Vehicle obstacle = new Vehicle(ColorRGBA.Yellow, this);
            obstacle.setLocalTranslation(((float) Math.random()) * 100 - 50f, 0, ((float) Math.random()) * 100f - 50f);
            obstacleNode.attachChild(obstacle);
        }

        amount = 40;
        // create neighbours
        for (int i = 0; i < amount; i++) {
            Vehicle neighbour = new Vehicle(ColorRGBA.Blue, this);
            neighbour.setLocalTranslation(((float) Math.random()) * 5f, 0, ((float) Math.random()) * 5f);
            neighbour.addControl(new FlockerControl(this));
            friendNode.attachChild(neighbour);
        }

        Vehicle v = new Vehicle(ColorRGBA.LightGray, this);
        v.addControl(new WanderControl());
        vehicleNode.attachChild(v);

        vehicleNode.attachChild(obstacleNode);
        vehicleNode.attachChild(friendNode);
        rootNode.attachChild(vehicleNode);
    }

    @Override
    public void simpleInitApp() {
        bulletAppState = new BulletAppState();
        bulletAppState.setThreadingType(BulletAppState.ThreadingType.PARALLEL);
        stateManager.attach(bulletAppState);

        setupBasicKeys();
        setupCamera();
        createTerrain();
        createVehicles();
        initCrossHairs();
    }

    private void initCrossHairs() {
        guiFont = assetManager.loadFont("Interface/Fonts/Default.fnt");
        BitmapText ch = new BitmapText(guiFont, false);
        ch.setSize(guiFont.getCharSet().getRenderedSize() * 2);
        ch.setText("+"); // crosshairs
        ch.setLocalTranslation( // center
                settings.getWidth() / 2 - guiFont.getCharSet().getRenderedSize() / 3 * 2,
                settings.getHeight() / 2 + ch.getLineHeight() / 2, 0);
        guiNode.attachChild(ch);
    }

    @Override
    public void simpleUpdate(float tpf) {
    }

    /**
     * Get all obsticals in the scene
     */
    @Override
    public List<Obstacle> getObstacals() {
        List<Obstacle> obstacles = new ArrayList<Obstacle>();
        for (Spatial s : obstacleNode.getChildren()) {
            if (s instanceof Vehicle) {
                Vehicle v = (Vehicle) s;
                obstacles.add(v.toObstacle());
            }
        }
        return obstacles;
    }

    @Override
    public List<Obstacle> obstacalsNearby(AbstractVehicle source, float radius) {
        List<Obstacle> obstacles = new ArrayList<Obstacle>();
        float r2 = radius * radius;
        for (Spatial s : obstacleNode.getChildren()) {
            if (s instanceof Vehicle) {
                Vehicle v = (Vehicle) s;
                float d = source.getWorldTranslation().subtract(v.getWorldTranslation()).lengthSquared();
                if (d < r2) // if it is within the radius
                {
                    obstacles.add(v.toObstacle());
                }
            }
        }
        return obstacles;
    }

    /**
     * find all neighbours in the scene, within the radius
     */
    @Override
    public List<Obstacle> neighboursNearby(AbstractVehicle source, float radius) {
        List<Obstacle> neighbours = new ArrayList<Obstacle>();
        float r2 = radius * radius;
        for (Spatial s : friendNode.getChildren()) {
            if (s instanceof Vehicle && !s.equals(source)) {
                Vehicle v = (Vehicle) s;
                float d = source.getWorldTranslation().subtract(v.getWorldTranslation()).lengthSquared();
                if (d < r2) // if it is within the radius
                {
                    if (source.flock != null && source.flock.contains(v)) {
                        neighbours.add(v.toObstacle());
                    } else if (source.flock == null) {
                        neighbours.add(v.toObstacle());
                    }
                }
            }
        }
        return neighbours;
    }

    @Override
    public AssetManager getAssetManager() {
        return assetManager;
    }

    @Override
    public PhysicsSpace getPhysicsSpace() {
        return bulletAppState.getPhysicsSpace();
    }
    private Vector3f pos1;
    private Vector3f pos2;
    private Spatial g1;
    private Spatial g2;
    private ArrayList<Vehicle> navigator = new ArrayList<Vehicle>();

    @Override
    public void onAction(String name, boolean isPressed, float tpf) {

        if (isPressed) {
            if (name.equals("ResetVehicles")) {
                rootNode.detachChild(vehicleNode);
                createVehicles();
            } else if (name.equals("1")) {
                Ray ray = new Ray(cam.getLocation(), cam.getDirection());

                CollisionResults collisionResults = new CollisionResults();
                terrain.collideWith(ray, collisionResults);

                if (collisionResults.size() != 0) {
                    if (g1 != null) {
                        g1.removeFromParent();
                    }
                    pos1 = collisionResults.getClosestCollision().getContactPoint();
                    g1 = createSphereAtPoint(pos1, ColorRGBA.Orange);
                }
            } else if (name.equals("2")) {
                Ray ray = new Ray(cam.getLocation(), cam.getDirection());

                CollisionResults collisionResults = new CollisionResults();
                terrain.collideWith(ray, collisionResults);

                if (collisionResults.size() != 0) {
                    if (g2 != null) {
                        g2.removeFromParent();
                    }
                    pos2 = collisionResults.getClosestCollision().getContactPoint();
                    g2 = createSphereAtPoint(pos2, ColorRGBA.Green);
                }
            } else if (name.equals("3")) {
                if (pos1 != null && pos2 != null) {
                    Path navmesh1 = navmesh(pos1, pos2);
                    if (navmesh1 != null) {
                        for (Vehicle v : navigator) {
                            v.removeFromParent();
                        }
                        int amount = 1;
                        float random = 0f;
                        Flock<AbstractVehicle> f = new Flock<AbstractVehicle>();
                        for (int i = 0; i < amount; i++) {
                            Vehicle v = new Vehicle(ColorRGBA.White, this);
                            v.addControl(new PathFollowerControl(this, navmesh1));
                            v.maxSpeed = 10f;
                            v.speed = 10f;
                            v.setLocalTranslation(pos1.add(FastMath.nextRandomFloat() * random - random / 2, 0, FastMath.nextRandomFloat() * random - random / 2));
                            v.flock = f;
                            f.add(v);
                            navigator.add(v);
                            friendNode.attachChild(v);
                        }
                    }
                }
            }
        }
    }
}
