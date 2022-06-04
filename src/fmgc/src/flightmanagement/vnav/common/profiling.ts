export class CpuTimer {
    public static integrationTime: number = 0;

    public static initializationTime: number = 0;

    public static visitorTime: number = 0;

    public static integrationSteps: number = 0;

    public static cruiseAndDescentTime: number = 0;

    static reset() {
        this.integrationTime = 0;
        this.initializationTime = 0;
        this.visitorTime = 0;
        this.integrationSteps = 0;
        this.cruiseAndDescentTime = 0;
    }
}

export function measurePerformance<T>(func: () => T, onComplete: (time: number, result: T) => void): T {
    const start = performance.now();
    const result = func();
    const end = performance.now();

    onComplete(end - start, result);

    return result;
}
