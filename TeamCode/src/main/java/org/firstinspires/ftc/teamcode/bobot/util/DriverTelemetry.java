package org.firstinspires.ftc.teamcode.bobot.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Locale;

public class DriverTelemetry {
    public enum Color {
        WHITE("#FFFFFF"),
        GREEN("#00E676"),
        YELLOW("#FFEA00"),
        RED("#FF1744"),
        BLUE("#40C4FF");

        private final String hex;

        Color(String hex) {
            this.hex = hex;
        }
    }

    private final Telemetry ds;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();
    private int decimals = 2;

    public DriverTelemetry(Telemetry ds) {
        this.ds = ds;
        ds.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    public void setDecimals(int decimals) {
        this.decimals = decimals;
    }

    public void add(String key, Object value) {
        add(key, value, Color.WHITE);
    }

    public void add(String key, Object value, Color color) {
        String text = format(value);
        ds.addLine("<font color='" + color.hex + "'>" + key + ": " + text + "</font>");
        if (value instanceof Number) {
            packet.put(key, value);
        } else {
            packet.put(key, text);
        }
    }

    public void add(String key, double value, double warn, double bad) {
        Color color = value < warn ? Color.GREEN : value < bad ? Color.YELLOW : Color.RED;
        add(key, value, color);
    }

    public void status(String key, boolean ok) {
        add(key, ok ? "OK" : "NO", ok ? Color.GREEN : Color.RED);
    }

    public void header(String text) {
        ds.addLine("<b><font color='" + Color.BLUE.hex + "'>" + text + "</font></b>");
        packet.addLine(text);
    }

    public void update() {
        ds.update();
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
    }
    public TelemetryPacket getPacket() {
        return packet;
    }

    private String format(Object value) {
        if (value instanceof Double || value instanceof Float) {
            return String.format(Locale.US, "%." + decimals + "f", ((Number) value).doubleValue());
        }
        return String.valueOf(value);
    }
}