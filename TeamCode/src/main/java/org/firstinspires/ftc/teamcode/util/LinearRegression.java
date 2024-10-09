package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;
import java.util.Map;

public class LinearRegression {

    private double target;
    private Double[] vars;
    private Map<Double, Double[]>example;

    public LinearRegression(HashMap<Double, Double[]> example) {
        this.example = example;
        System.out.println("Atribuição Concluída");
    }

    public void train(double trainSize){

    }
}