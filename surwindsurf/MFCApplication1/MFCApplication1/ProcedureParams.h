#pragma once

// 流程参数结构定义
// 用于配置7步穿刺流程的所有参数

struct ProcedureParams
{
	// ========== 步骤2：激光粗调参数 ==========
	float fTargetHeight;        // 目标高度（默认200mm，后续从RealSense动态获取）
	float fCoarseTolerance;     // 粗调精度（默认±2mm）
	
	// ========== 步骤4：精准对位参数 ==========
	float fFineTolerance;       // 精调精度（默认±0.2mm）
	int nMaxAdjustCount;        // 最大调整次数（默认50次）

	// ========== 步骤4：XY闭环参数（IR像素闭环，后续接入RealSense） ==========
	BOOL bEnableXYAlign;        // 是否启用XY闭环（默认FALSE，避免未集成视觉时误触发）
	float fMmPerPixelX;         // X方向mm/px标定
	float fMmPerPixelY;         // Y方向mm/px标定
	int nXYPixelThreshold;      // XY收敛像素阈值（|dx|,|dy|）
	float fXYMaxStepMm;         // 单次XY最大移动量（mm）
	float fXYMinStepMm;         // 单次XY最小移动量（mm）
	int nXYMaxIters;            // XY最大迭代次数
	
	// ========== 步骤5：消毒参数 ==========
	float fDisinfectTime;       // 消毒时间（默认2秒）
	
	// ========== 步骤6：穿刺参数 ==========
	float fPunctureDepth;       // 穿刺深度（默认50mm）
	float fPunctureSpeed;       // 穿刺速度（默认50%）
	
	// ========== 安全参数 ==========
	float fSafeHeight;          // 安全高度（默认100mm）
	BOOL bEnableVision;         // 是否启用视觉系统（默认FALSE）
	
	// ========== 超时设置 ==========
	int nStepTimeout;           // 单步超时时间（秒，默认30秒）
	
	// 构造函数：设置默认值
	ProcedureParams()
	{
		// 步骤2：激光粗调
		fTargetHeight = 100.0f;      // 默认100mm（可由外部/RealSense动态覆盖）
		fCoarseTolerance = 2.0f;     // ±2mm
		
		// 步骤4：精准对位
		fFineTolerance = 0.2f;       // ±0.2mm
		nMaxAdjustCount = 50;        // 最大50次调整

		// 步骤4：XY闭环（默认关闭，待集成IR检测后启用）
		bEnableXYAlign = FALSE;
		fMmPerPixelX = 0.05f;        // 仅占位默认值：0.05mm/px
		fMmPerPixelY = 0.05f;
		nXYPixelThreshold = 3;       // 2~3px为常见阈值
		fXYMaxStepMm = 0.5f;
		fXYMinStepMm = 0.05f;
		nXYMaxIters = 30;
		
		// 步骤5：消毒
		fDisinfectTime = 2.0f;       // 2秒
		
		// 步骤6：穿刺
		fPunctureDepth = 50.0f;      // 50mm
		fPunctureSpeed = 50.0f;      // 50%速度
		
		// 安全参数
		fSafeHeight = 0.0f;          // 100mm安全高度
		bEnableVision = FALSE;       // 默认不启用视觉
		
		// 超时设置
		nStepTimeout = 30;           // 30秒超时
	}
	
	// 验证参数有效性
	BOOL Validate() const
	{
		// 检查高度参数的有效范围（支持1000mm等大值）
		if (fTargetHeight < 0 || fTargetHeight > 2000) return FALSE;  // 支持0-2000mm
		if (fSafeHeight < 0 || fSafeHeight > 2000) return FALSE;      // 支持0-2000mm
		
		// ★ 新增：检查目标高度是否在激光测距范围内
		// 激光测距仪范围：120-280mm（零点200mm）
		const float LASER_MIN = 120.0f;
		const float LASER_MAX = 280.0f;
		if (fTargetHeight < LASER_MIN || fTargetHeight > LASER_MAX)
		{
			// 警告但不阻止（可能用于其他场景）
			// return FALSE; // 可选：严格模式下取消注释
		}
		
		// 检查精度参数
		if (fCoarseTolerance <= 0 || fCoarseTolerance > 50) return FALSE;  // 支持0-50mm
		if (fFineTolerance <= 0 || fFineTolerance > 10) return FALSE;       // 支持0-10mm
		
		// ★ 新增：检查精度参数的逻辑合理性
		if (fFineTolerance >= fCoarseTolerance)
		{
			// 精调精度应小于粗调精度
			return FALSE;
		}
		
		// ★ 新增：检查安全高度的逻辑合理性
		if (fSafeHeight > 0 && fSafeHeight <= fFineTolerance)
		{
			// 安全高度应大于精调精度（避免碰撞）
			return FALSE;
		}
		
		// 检查时间参数
		if (fDisinfectTime < 0 || fDisinfectTime > 60) return FALSE;
		if (nStepTimeout < 1 || nStepTimeout > 300) return FALSE;
		
		// 检查穿刺参数（扩大范围支持1000mm等大值）
		if (fPunctureDepth < 0 || fPunctureDepth > 2000) return FALSE;  // 支持0-2000mm（1000单位约50mm）
		if (fPunctureSpeed < 0 || fPunctureSpeed > 100) return FALSE;
		
		// ★ 新增：检查穿刺深度必须大于0
		if (fPunctureDepth <= 0)
		{
			return FALSE;
		}
		
		// ★ 新增：检查调整次数
		if (nMaxAdjustCount < 1 || nMaxAdjustCount > 200)
		{
			return FALSE;
		}

		// Step4：XY闭环参数校验（启用时才严格校验）
		if (bEnableXYAlign)
		{
			if (fMmPerPixelX <= 0.0f || fMmPerPixelX > 5.0f) return FALSE;
			if (fMmPerPixelY <= 0.0f || fMmPerPixelY > 5.0f) return FALSE;
			if (nXYPixelThreshold < 1 || nXYPixelThreshold > 50) return FALSE;
			if (fXYMaxStepMm <= 0.0f || fXYMaxStepMm > 10.0f) return FALSE;
			if (fXYMinStepMm <= 0.0f || fXYMinStepMm > fXYMaxStepMm) return FALSE;
			if (nXYMaxIters < 1 || nXYMaxIters > 200) return FALSE;
		}
		
		return TRUE;
	}
};
