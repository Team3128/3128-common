package common.narwhaldashboard;

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

import common.utility.Log;
import edu.wpi.first.util.function.BooleanConsumer;

public class NarwhalDashboard extends WebSocketServer {
    private final HashMap<String, Supplier<Object>> updateMap = new HashMap<String, Supplier<Object>>();
    private final HashMap<String, List<Object>> initMap = new HashMap<String, List<Object>>();
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

    public static void setPort(int port) {
        PORT = port;
    }

    /**
     * Starts the NarwhalDashboard server. This opens it up to be able to be
     * connected to by client devices (the DS Laptop, a tablet controller, etc) and
     * begins streaming data.
     */
    public static void startServer() {
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

    private NarwhalDashboard(int port) throws UnknownHostException {
        super(new InetSocketAddress(port));
    }

    private void initDashboard() {
        addUpdate("selectedAuto", ()-> selectedAuto);
        addAction("selectAuto", autoName -> selectAuto(autoName[0]));
        addAction("button", button -> updateButton(button[0], button[1].equals("down")));
    }

    public void addInit(String key, List<Object> objects) {
        initMap.put(key, objects);
    }

    public void addInit(String key, Object object) {
        addInit(key, Arrays.asList(object));
    }
    /**
     * Adds an autonomous program to NarwhalDashboard's auto picker
     * 
     * @param names    - The human-readable name of the autonomous program
     */
    @SuppressWarnings("all")
    public void addAutos(String... names) {
        autoPrograms.addAll(Arrays.asList(names));
        addInit("auto", Arrays.asList(names));
    }

    // Called once on connection with web server
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

    @SuppressWarnings("unchecked")
    public void update() {
        if (conn == null) return;

        if (conn.isOpen()) {
            final JSONObject obj = new JSONObject();
            for (final String key : updateMap.keySet()) {
                obj.put(key, updateMap.get(key).get());
            }
            conn.send(obj.toJSONString());
        }
    }

    public void addUpdate(String key, Supplier<Object> obj) {
        updateMap.put(key, obj);
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
    }

    public void addButton(String key, BooleanConsumer button) {
        buttons.put(key, button);
    } 

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

    private void updateButton(String key, boolean down) {
        if (!buttons.containsKey(key)) {
            Log.recoverable("NarwhalDashboard", "Button \"" + key + "\" was never added.");
            return;
        }
        buttons.get(key).accept(down);
    }

    public void addAction(String key, Consumer<String[]> action) {
        actionMap.put(key, action);
    }

    // Called by request from web server 
    @Override
    public void onMessage(WebSocket conn, String message) {
        Log.info("NarwhalDashboard", message);
        final String[] parts = message.split(":");

        if (!actionMap.containsKey(parts[0])) return;

        final Consumer<String[]> action = actionMap.get(parts[0]);

        action.accept(Arrays.copyOfRange(parts, 1, parts.length));
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        ex.printStackTrace();
    }

    @Override
    public void onStart() {}

}
