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
import org.java_websocket.exceptions.WebsocketNotConnectedException;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

import common.core.misc.NAR_Robot;
import common.hardware.motorcontroller.DeviceState;
import common.utility.Log;
import edu.wpi.first.util.function.BooleanConsumer;

/**
 * The server for NarwhalDashboard, Team 3128's own web dashboard. It sends robot data to the
 * dashboard page, receives its button presses, and holds the list of selectable autos.
 *
 * <p>It runs a WebSocket server on the robot (port 5805). Robot code registers values to display
 * with {@link #addUpdate} and buttons with {@link #addButton}; the dashboard connects and
 * exchanges JSON messages with it.
 *
 * <p>The server starts the first time {@link #getInstance()} is called.
 *
 * <p>This class is being replaced by Elastic. Robot code should use
 * {@link common.utility.dashboard.Dashboard} instead of calling it directly. See
 * {@code README.md} in the {@code utility} package for the plan.
 *
 * @deprecated Use {@link common.utility.dashboard.Dashboard}, which shows everything on Elastic.
 * @since Deep Space 2019
 * @author Mason Lam
 */
@Deprecated
public class NarwhalDashboard extends WebSocketServer implements AutoCloseable {
    private final HashMap<String, List<Object>> initMap = new HashMap<String, List<Object>>();
    private final HashMap<String, Supplier<Object>> updateMap = new HashMap<String, Supplier<Object>>();
    private final HashMap<String, Consumer<String[]>> actionMap = new HashMap<String, Consumer<String[]>>();

    private final HashMap<String, BooleanConsumer> buttons = new HashMap<String, BooleanConsumer>();

    private final ArrayList<String> autoPrograms = new ArrayList<String>();
    /** Set by the server thread when the page selects an auto, and read by the robot thread. */
    private volatile String selectedAuto;

    private volatile WebSocket conn;

    private static NarwhalDashboard instance;

    private static int PORT = 5805;

    /** Returns the single dashboard server, starting it on the first call. */
    public static synchronized NarwhalDashboard getInstance() {
        if (instance == null) {
            startServer();
        }
        return instance;
    }

    /**
     * Starts the NarwhalDashboard server, allowing client devices (the Driver Station laptop or a
     * tablet, for example) to connect, and
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
     * Sets the port number.
     * @param port The port number
     */
    public static void setPort(int port) {
        PORT = port;
    }

    /**
     * Creates a NarwhalDashboard server.
     * @param port The port number
     * @throws UnknownHostException If the IP address is invalid
     */
    private NarwhalDashboard(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
        NAR_Robot.addPeriodic(this::update, 0.02);
    }

    /**
     * Returns the auto selected on the web page.
     * @return The name of the selected auto
     */
    public String getSelectedAuto() {
        return selectedAuto;
    }

    /**
     * Initializes NarwhalDashboard autos and buttons.
     */
    private void initDashboard() {
        addUpdate("selectedAuto", ()-> selectedAuto);
        addAction("selectAuto", autoName -> selectAuto(autoName[0]));
        addAction("button", button -> updateButton(button[0], button[1].equals("true")));
    }

    /**
     * Sends a list of objects to the web page on initialization.
     * @param key The name of the objects on the dashboard
     * @param objects The objects to add
     */
    public void addInit(String key, List<Object> objects) {
        initMap.put(key, objects);
    }

    /**
     * Sends an object to the web page on initialization.
     * @param key The name of the object on the dashboard
     * @param object The object to add
     */
    public void addInit(String key, Object object) {
        addInit(key, Arrays.asList(object));
    }
    /**
     * Adds an autonomous program to NarwhalDashboard's auto chooser.
     * 
     * @param names The human-readable names of the autonomous programs
     */
    public void addAutos(String... names) {
        autoPrograms.addAll(Arrays.asList(names));
        addInit("auto", Arrays.asList((Object[]) names));
    }

    /** 
     * Called after a client connects to the web server.
	 * @param conn The WebSocket instance on which this event occurred
	 * @param handshake The handshake of the WebSocket instance
	 */
    @SuppressWarnings("unchecked")
    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        Log.info("NarwhalDashboard", conn.getRemoteSocketAddress().getHostName() + " has opened a connection.");

        this.conn = conn;

        final JSONObject obj = new JSONObject();
        for (final String key : initMap.keySet()) {
            final List<Object> values = initMap.get(key);
            if (values.size() <= 1) {
                obj.put(key, values.get(0));
                continue;
            }

            final JSONArray arr = new JSONArray();
            for (final Object value : values) {
                arr.add(value);
            }
            obj.put(key, arr);
        }

        conn.send(obj.toJSONString());
    }

    /**
     * Sends an object to the web page on every update.
     * @param key The name of the object
     * @param obj The object to add
     */
    public void addUpdate(String key, Supplier<Object> obj) {
        updateMap.put(key, obj);
    }

    /**
     * Sends a message to console.log on NarwhalDashboard.
     * @param message The message to log
     */
    @SuppressWarnings("unchecked")
    public void sendMessage(String message) {
        WebSocket localConn = conn;

        if (localConn == null) return;
        final JSONObject obj = new JSONObject();
        obj.put("Message", message);
        try {
            localConn.send(obj.toJSONString());
        } catch (Exception e) {
            Log.info("Dashboard", e.toString());
            conn.close();
        }
    }

    /**
     * Sends updated data to the web page.
     */
    @SuppressWarnings("unchecked")
    private void update() {
        WebSocket localConn = conn;

        if (localConn == null) return;

        if (localConn.isOpen()) {
            final JSONObject obj = new JSONObject();
            for (final String key : updateMap.keySet()) {
                obj.put(key, updateMap.get(key).get());
            }
            try {
                localConn.send(obj.toJSONString());
            } catch (Exception e) {
                Log.info("Dashboard", e.toString());
                conn.close();
            }
        }
    }

    /**
     * Adds an action to run when a message arrives from the web page.
     * @param key The name of the action
     * @param action The action to run
     */
    public void addAction(String key, Consumer<String[]> action) {
        actionMap.put(key, action);
    }

    /**
     * Assigns an action to a NarwhalDashboard button.
     * @param key The name of the button
     * @param button The action to run when the button is pressed
     */
    public void addButton(String key, BooleanConsumer button) {
        buttons.put(key, button);
    }

    /**
	 * Called when a message arrives from the web page.
	 * @param conn The WebSocket instance on which this event occurred
	 * @param message The UTF-8 decoded message
	 */
    @Override
    public void onMessage(WebSocket conn, String message) {
        Log.info("NarwhalDashboard", message);
        // Message format: category:key:value or category:value, for example, auto:"exampleAuto"
        final String[] parts = message.split(":");

        if (!actionMap.containsKey(parts[0])) return;

        final Consumer<String[]> action = actionMap.get(parts[0]);

        action.accept(Arrays.copyOfRange(parts, 1, parts.length));
    }

    /**
     * Selects the auto.
     * @param autoName The name of the auto
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
     * Changes the state of a button.
     * @param key The name of the button
     * @param down The state of the button
     */
    private void updateButton(String key, boolean down) {
        if (!buttons.containsKey(key)) {
            Log.recoverable("NarwhalDashboard", "Button \"" + key + "\" was never added.");
            return;
        }
        buttons.get(key).accept(down);
    }

    /** Displays the health of a part of the robot, for example, a motor's {@code getState()}, as an indicator named {@code name}. */
    public void checkState(String name, Supplier<DeviceState> state) {
        addUpdate(name, ()-> state.get().toString());
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        ex.printStackTrace();
    }

    @Override
    public void onStart() {}

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        this.conn = null;
    }

    /**
     * Closes the dashboard.
     */
    @Override
    public void close() {
        conn = null;
    }
}
