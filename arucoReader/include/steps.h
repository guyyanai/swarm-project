#ifndef STEPS_DEFINED
#define STEPS_DEFINED

struct steps {
    int stepX;
    int stepY;
    int stepZ;
    int stepT;
    steps(int stepX, int stepY, int stepZ, int stepT) : stepX(stepX), stepY(stepY), stepZ(stepZ), stepT(stepT) {}
    steps operator+(const steps& other) {
        return steps(stepX + other.stepX, stepY + other.stepY, stepZ + other.stepZ, stepT + other.stepT);
    }
    steps operator/(int d) {
        return steps(stepX / d, stepY / d, stepZ / d, stepT / d);
    }
};

#endif // STEPS_DEFINED