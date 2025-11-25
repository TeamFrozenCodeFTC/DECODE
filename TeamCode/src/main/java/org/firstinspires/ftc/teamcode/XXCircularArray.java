package org.firstinspires.ftc.teamcode;

import java.util.Iterator;
import java.util.NoSuchElementException;

public class XXCircularArray<E> implements Iterable<E> {
    int size;
    int head;
    int tail;
    E[] elements;

    @SuppressWarnings("unchecked")
    public XXCircularArray(int capacity) {
        this.elements = (E[]) new Object[capacity];
        clear();
    }

    private int nextIndex(int currentIndex) {
        int next = currentIndex + 1;
        if (next >= elements.length) {
            next = 0;
        }
        return next;
    }

    private int previousIndex(int currentIndex) {
        int prev = currentIndex - 1;
        if (prev < 0) {
            prev = elements.length - 1;
        }
        return prev;
    }

    public int size() {
        return this.size;
    }

    public void add(E element) {
        head = previousIndex(head);

        if (size < elements.length) {
            // Size will never be greater than the array length
            size++;
            if (tail == -1) {
                tail = head;
            }
        } else {
            tail = previousIndex(tail);
        }
        elements[head] = element;
    }

    public void append(E element) {
        tail = nextIndex(tail);

        if (size < elements.length) {
            // Size will never be greater than the array length
            size++;
            if (head == -1) {
                head = tail;
            }
        } else {
            head = nextIndex(head);
        }

        elements[tail] = element;
    }

    public E getFirst() {
        if (this.size < 1) {
            throw new NoSuchElementException("CircularArray is empty");
        }
        return elements[this.head];
    }

    public E getLast() {
        if (this.size < 1) {
            throw new NoSuchElementException("CircularArray is empty");
        }
        return elements[this.tail];
    }

    public void clear() {
        this.size = 0;
        this.head = -1;
        this.tail = -1;
    }

    class CircularArrayInterator<T> implements Iterator<T> {
        int iterIndex;
        int elementsProcessed;

        public CircularArrayInterator(int head) {
            this.iterIndex = head;
            this.elementsProcessed = 0;
        }

        @Override
        public boolean hasNext() {
            return this.elementsProcessed < XXCircularArray.this.size;
        }

        @Override
        public T next() {
            if (!hasNext()) {
                throw new NoSuchElementException();
            }

            @SuppressWarnings("unchecked")
            T element = (T) elements[this.iterIndex];
            this.iterIndex = nextIndex(this.iterIndex);
            this.elementsProcessed++;

            return element;
        }
    }

    @Override
    public Iterator<E> iterator() {
        return new CircularArrayInterator<E>(this.head);
    }
}
