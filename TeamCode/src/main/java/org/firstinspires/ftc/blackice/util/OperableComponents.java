package org.firstinspires.ftc.blackice.util;

/**
 * A group of immutable components that can be operated on through
 * adding, subtracting, multiplying, or any other custom operation.
 */
public abstract class OperableComponents<Subclass extends OperableComponents<Subclass>> {
    @FunctionalInterface
    public interface ComponentOperator {
        double applyToComponent(double component);
    }
    
    @FunctionalInterface
    public interface ComponentPairOperator {
        double applyToComponents(double component1, double component2);
    }
    
    @FunctionalInterface
    private interface IndexToComponent {
        double toComponent(int index);
    }
    
    protected final double[] components;
    
    protected OperableComponents(double[] components) {
        this.components = components;
    }
    
    /**
     * Apply a custom operation to each component.
     * <code>new Vector(operation(x), operation(y))</code>
     */
    public Subclass map(
        ComponentOperator operator
    ) {
        return mapThroughIndex((i) -> operator.applyToComponent(components[i]));
    }
    
    /**
     * Apply a custom operation to each pair of components.
     * <code>new Vector(operation(x1, x2), operation(y1, y2))</code>
     */
    public Subclass map(
        Subclass other,
        ComponentPairOperator operator
    ) {
        return mapThroughIndex((i) ->
            operator.applyToComponents(this.components[i], other.components[i]));
    }
    
    public Subclass plus(double scalar) {
        return this.map(component -> component + scalar);
    }
    public Subclass minus(double scalar) {
        return this.map(component -> component - scalar);
    }
    public Subclass times(double scalar) {
        return this.map(component -> component * scalar);
    }
    public Subclass divide(double scalar) {
        return this.map(component -> component / scalar);
    }
    
    public Subclass plus(Subclass other) {
        return this.map(other, Double::sum);
    }
    public Subclass minus(Subclass other) {
        return this.map(other,
            (component1, component2) -> component1 - component2);
    }
    public Subclass times(Subclass other) {
        return this.map(other,
            (component1, component2) -> component1 * component2);
    }
    public Subclass divide(Subclass other) {
        return this.map(other,
            (component1, component2) -> component1 / component2);
    }
    
    protected abstract Subclass fromComponentArray(double[] components);
    
    /**
     * Get the absolute value of the largest component.
     */
    protected double getMaxAbs() {
        double max = 0.0;
        for (double component : components) {
            double abs = Math.abs(component);
            if (abs > max) {
                max = abs;
            }
        }
        return max;
    }
    
    private Subclass mapThroughIndex(IndexToComponent indexToComponent) {
        double[] result = new double[components.length];
        for (int i = 0; i < components.length; i++) {
            result[i] = indexToComponent.toComponent(i);
        }
        return fromComponentArray(result);
    }
}
