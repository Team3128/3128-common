package common.utility.narwhaldashboard;

import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import common.core.misc.NAR_Robot;
import common.utility.Log;
import edu.wpi.first.util.function.BooleanConsumer;

/**
 * Team 3128's {@link WebSocketServer} class, used to log robot data and select autos
 * @since Deep Space 2019
 * @author Mason Lam
 */
public class NarwhalDashboard extends WebSocketServer implements AutoCloseable {
    private final HashMap<String, List<Object>> initMap = new HashMap<String, List<Object>>();
    private final HashMap<String, Supplier<Object>> updateMap = new HashMap<String, Supplier<Object>>();
    private final HashMap<String, Consumer<String[]>> actionMap = new HashMap<String, Consumer<String[]>>();

    private final HashMap<String, BooleanConsumer> buttons = new HashMap<String, BooleanConsumer>();

    private final ArrayList<String> autoPrograms = new ArrayList<String>();
    private String selectedAuto;

    private WebSocket conn;

    private static NarwhalDashboard instance;

    private static int PORT = 5805;

    public static synchronized NarwhalDashboard getInstance() {
        if (instance == null) {
            startServer();
        }
        return instance;
    }

    /**
     * Starts the NarwhalDashboard server. This opens it up to be able to be
     * connected to by client devices (the DS Laptop, a tablet controller, etc) and
     * begins streaming data.
     */
    private static void startServer() {
        try {
            instance = new NarwhalDashboard(PORT);
            instance.setReuseAddr(true);
            instance.start();
            instance.initDashboard();

            Log.info("NarwhalDashboard", "Server has started on port " + PORT);
        } catch (UnknownHostException e) {
            e.printStackTrace();
        }
    }

    /**
     * Sets the IP address' port number
     * @param port the port number
     */
    public static void setPort(int port) {
        PORT = port;
    }

    /**
     * Creates a new NarwhalDashboard server
     * @param port the port number
     * @throws UnknownHostException invalid IP address
     */
    private NarwhalDashboard(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
        NAR_Robot.addPeriodic(this::update, 0.1, 0.01);
    }

    /**
     * Returns the selectedAuto on the web server
     * @return A string containing an auto
     */
    public String getSelectedAuto() {
        return selectedAuto;
    }

    /**
     * Initializes NarwhalDashboard autos and buttons
     */
    private void initDashboard() {
        addUpdate("selectedAuto", ()-> selectedAuto);
        addAction("selectAuto", autoName -> selectAuto(autoName[0]));
        addAction("button", button -> updateButton(button[0], button[1].equals("true")));
    }

    /**
     * Sends a list of objects to the web server on intialize
     * @param key Name of the objects on dashboard
     * @param objects List of objects added
     */
    public void addInit(String key, List<Object> objects) {
        initMap.put(key, objects);
    }

    /**
     * Sends an object to the web server on intialize
     * @param key Name of the object on dashboard
     * @param object - Object added
     */
    public void addInit(String key, Object object) {
        addInit(key, Arrays.asList(object));
    }
    /**
     * Adds an autonomous program to NarwhalDashboard's auto picker
     * 
     * @param names - The human-readable name of the autonomous program
     */
    @SuppressWarnings("all")    //I cannot figure out what warning this is
    public void addAutos(String... names) {
        autoPrograms.addAll(Arrays.asList(names));
        addInit("auto", Arrays.asList(names));
    }

    /** 
     * Called after connecting to the web server
	 * @param conn The WebSocket instance this event is occuring on.
	 * @param handshake The handshake of the websocket instance
	 */
    @SuppressWarnings("unchecked")
    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        Log.info("NarwhalDashboard", conn.getRemoteSocketAddress().getHostName() + " has opened a connection.");

        this.conn = conn;

        final JSONObject obj = new JSONObject();
        //Sends every object to NarwhalDashboard on initialize
        for (final String key : initMap.keySet()) {
            final List<Object> values = initMap.get(key);
            //Sends a singular object
            if (values.size() <= 1) {
                obj.put(key, values.get(0));
                continue;
            }

            //Sends a list of objects
            final JSONArray arr = new JSONArray();
            for (final Object value : values) {
                arr.add(value);
            }
            obj.put(key, arr);
        }

        conn.send(obj.toJSONString());
    }

    /**
     * Sends an object to the web server every update
     * @param key Name of the object
     * @param obj Object added
     */
    public void addUpdate(String key, Supplier<Object> obj) {
        updateMap.put(key, obj);
    }

    /**
     * Sends a message to console.log on Narwhal Dashboard.
     * @param message Message to be logged on dashboard
     */
    @SuppressWarnings("unchecked")
    public void sendMessage(String message) {
        final JSONObject obj = new JSONObject();
        obj.put("Message", message);
        conn.send(obj.toJSONString());
    }

    /**
     * Updates NarwhalDashboard sending data to the web server
     */
    @SuppressWarnings("unchecked")
    private void update() {
        if (conn == null) return;

        if (conn.isOpen()) {
            //Sends data as a JSON
            final JSONObject obj = new JSONObject();
            for (final String key : updateMap.keySet()) {
                obj.put(key, updateMap.get(key).get());
            }
            conn.send(obj.toJSONString());
        }
    }

    /**
     * Adds an action to be run on a message from the web server
     * @param key Name of the action
     * @param action - Action to be run
     */
    public void addAction(String key, Consumer<String[]> action) {
        actionMap.put(key, action);
    }

    /**
     * Assigns an action to a NarwhalDashboard button
     * @param key Name of the button
     * @param button Action to be run on button press
     */
    public void addButton(String key, BooleanConsumer button) {
        buttons.put(key, button);
    }

    /**
	 * Called when a message from the web server is sent
	 * @param conn The WebSocket instance this event is occurring on.
	 * @param message The UTF-8 decoded message that was received.
	 */
    @Override
    public void onMessage(WebSocket conn, String message) {
        Log.info("NarwhalDashboard", message);
        //Message format category + key + value or category + value, example auto:"exampleAuto"
        final String[] parts = message.split(":");

        if (!actionMap.containsKey(parts[0])) return;

        final Consumer<String[]> action = actionMap.get(parts[0]);

        action.accept(Arrays.copyOfRange(parts, 1, parts.length));
    }

    /**
     * Selects the auto
     * @param autoName Name of the auto
     */
    private void selectAuto(String autoName) {
        if (autoName == null) {
            selectedAuto = null;
            return;
        }
        if (autoPrograms.contains(autoName)) {
            selectedAuto = autoName;
            Log.info("NarwhalDashboard", "Selected auto program \"" + selectedAuto + "\"");
            return;
        }
        selectedAuto = null;
        Log.recoverable("NarwhalDashboard", "Auto program \"" + autoName + "\" does not exist.");
    }

    /**
     * Changes a button state
     * @param key Name of the button
     * @param down State of the button
     */
    private void updateButton(String key, boolean down) {
        if (!buttons.containsKey(key)) {
            Log.recoverable("NarwhalDashboard", "Button \"" + key + "\" was never added.");
            return;
        }
        buttons.get(key).accept(down);
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        ex.printStackTrace();
    }

    @Override
    public void onStart() {}

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {}

    /**
     * Closes the dashboard.
     */
    @Override
    public void close() {
        instance = null;
    }
}
