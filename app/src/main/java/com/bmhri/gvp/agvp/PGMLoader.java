package com.bmhri.gvp.agvp;

import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.IOException;

/**
 * Created by shydh on 10/19/17.
 */

public class PGMLoader {
    private char ch0, ch1;
    private int width, height;
    private int max_pix;

    public void readPGMHeader(String name) {
        DataInputStream in;

        try {
            FileInputStream fin = new FileInputStream(name);
            in = new DataInputStream(fin);
            ch0 = (char) in.readByte();
            ch1 = (char) in.readByte();
            if (ch0 != 'P' || ch1 != '5') {
                System.out.print("Not a pgm image!" + " [0]=" + ch0 + ", [1]=" + ch1);
                System.exit(0);
            }
            in.readByte();                  //读空格  
            char c = (char) in.readByte();

            if (c == '#')                    //读注释行  
            {
                do {
                    c = (char) in.readByte();
                } while ((c != '\n') && (c != '\r'));
                c = (char) in.readByte();
            }

            //读出宽度  
            if (c < '0' || c > '9') {
                System.out.print("Errow!");
                System.exit(1);
            }

            int k = 0;
            do {
                k = k * 10 + c - '0';
                c = (char) in.readByte();
            } while (c >= '0' && c <= '9');
            width = k;

            //读出高度  
            c = (char) in.readByte();
            if (c < '0' || c > '9') {
                System.out.print("Errow!");
                System.exit(1);
            }

            k = 0;
            do {
                k = k * 10 + c - '0';
                c = (char) in.readByte();
            } while (c >= '0' && c <= '9');
            height = k;

            //读出灰度最大值(尚未使用)  
            c = (char) in.readByte();
            if (c < '0' || c > '9') {
                System.out.print("Errow!");
                System.exit(1);
            }

            k = 0;
            do {
                k = k * 10 + c - '0';
                c = (char) in.readByte();
            } while (c >= '0' && c <= '9');
            max_pix = k;
        } catch (IOException e1) {
            System.out.println("Exception!");
        }
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }
}
