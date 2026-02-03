#include <stdio.h>
#include <stdlib.h> // 包含 standard library，便于在所有环境中编译

int main() {
    
    // --- 实验常量定义 ---
    const int N_FLOORS = 64;    // 总楼层数
    const int F_TRUE = 38;       // 鸡蛋真正的耐摔值
    
    // --- 成本系数定义 ---
    // 物质匮乏时期 (p1=2, p2=1, p3=4)
    const int P1_SCARCITY = 2; 
    const int P2_SCARCITY = 1; 
    const int P3_SCARCITY = 4; 
    
    // 人力成本增长时期 (p1=4, p2=1, p3=2)
    const int P1_LABOR    = 4;  
    const int P2_LABOR    = 1;  
    const int P3_LABOR    = 2;  

    // --- 实验指标变量 (对应 CPU 寄存器或内存中的 m, n, h, count) ---
    int total_up_floors = 0;   // m (上的楼层总数)
    int total_down_floors = 0; // n (下的楼层总数)
    int broken_eggs = 0;       // h (摔破的鸡蛋总数)
    int total_throws = 0;      // count (摔的总次数)

    // --- 算法变量 ---
    int high = N_FLOORS;     // 搜索区间上界
    int low = 1;             // 搜索区间下界
    int last_floor = N_FLOORS; // 初始站在最高层 (128)
    int floor_to_test = 0;   // 当前试探楼层
    int result_F = 0;        // 最终找到的耐摔值

    // ------------------------------------------------
    // Phase 1: 二分查找循环 (直到 high - low < 2)
    // ------------------------------------------------
    while ((high - low) >= 2) {
        total_throws++;
        
        // 核心计算: floor_to_test = (high + low) / 2
        floor_to_test = (high + low) / 2;

        // 成本计算 m 和 n (无 abs)
        int diff = floor_to_test - last_floor; // 计算楼层差值
        
        if (diff > 0) {
            // diff > 0 (上楼): 累加 m
            total_up_floors += diff;
        } else {
            // diff <= 0 (下楼或原地): 累加 n
            int abs_diff = 0 - diff; // 0 - diff 得到正的绝对值
            total_down_floors += abs_diff;
        }
        last_floor = floor_to_test; // 更新上次楼层

        // 模拟摔蛋 (test_egg: floor_to_test > F_TRUE)
        if (floor_to_test > F_TRUE) {
            broken_eggs++; 
            high = floor_to_test - 1;
        } else {
            low = floor_to_test;
        }
    }

    // ------------------------------------------------
    // Phase 2: 最终确认 (处理 high - low = 0 或 1 的情况)
    // ------------------------------------------------
    if (high == low) {
        result_F = high;
    }
    else { // high = low + 1, 需试摔 high 楼层
        total_throws++;
        
        // 成本计算 (测试 high 楼层)
        int diff = high - last_floor;
        if (diff > 0) {
            total_up_floors += diff;
        } else {
            int abs_diff = 0 - diff;
            total_down_floors += abs_diff;
        }
        
        // 最终试摔 high 楼层
        if (high > F_TRUE) {
            broken_eggs++; 
            result_F = low; // 碎了，耐摔值是下一层 (low)
        } else {
            result_F = high; // 没碎，耐摔值就是 high
        }
    }

    // ------------------------------------------------
    // 结果输出与成本计算
    // ------------------------------------------------
    printf("--- 比萨塔摔鸡蛋模型验证 ---\n");
    printf("总楼层 N: %d, 真实耐摔值 F_true: %d\n", N_FLOORS, F_TRUE);
    printf("找到的耐摔楼层 F: %d\n", result_F);
    
    printf("\n--- 实验指标 ---\n");
    printf("摔的总次数: %d\n", total_throws);
    printf("摔破的鸡蛋总数 h: %d\n", broken_eggs);
    printf("上的楼层总数 m: %d\n", total_up_floors);
    printf("下的楼层总数 n: %d\n", total_down_floors);

    // 成本计算 f = m*p1 + n*p2 + h*p3
    long m_val = total_up_floors;
    long n_val = total_down_floors;
    long h_val = broken_eggs;

    // 成本分析 - 物质匮乏时期
    long cost_scarcity = m_val * P1_SCARCITY + n_val * P2_SCARCITY + h_val * P3_SCARCITY;
    printf("\n--- 成本分析 (f = m*p1 + n*p2 + h*p3) ---\n");
    printf("物质匮乏时期 (p1=2, p2=1, p3=4) 总成本: %ld\n", cost_scarcity);

    // 成本分析 - 人力成本增长时期
    long cost_labor = m_val * P1_LABOR + n_val * P2_LABOR + h_val * P3_LABOR;
    printf("人力成本增长时期 (p1=4, p2=1, p3=2) 总成本: %ld\n", cost_labor);

    return 0;
}
