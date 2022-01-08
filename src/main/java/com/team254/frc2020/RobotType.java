package com.team254.frc2020;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class RobotType {
    public enum Type {
        COMPETITION,
        PRACTICE
    }

    private static String kPracticeMAC = "00-80-2F-25-B4-23";

    public static Type getRobotType() {
        String mac = getMACAddress();
        if (mac != null && !mac.equals("") && mac.equals(kPracticeMAC)) {
            return Type.PRACTICE;
        }
        return Type.COMPETITION;
    }


    /**
     * @return the MAC address of the robot
     */
    private static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }

        return "";
    }
}
