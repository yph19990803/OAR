using UnityEngine;
using System.Collections.Generic;

// Opportunity-Aware RDW Controller 的第一版验证实现。
// 该控制器保留了 OpenRDW 原有的注入路径，但在注入之前插入了一层轻量级决策层：
// 1）收集一小段时间窗口内的状态历史
// 2）估计机会度 / 可转向性 / 增益预算
// 3）在安全性与平滑性约束下调度增益
// 4）通过现有 API 注入平移 / 旋转 / 曲率增益
public class OpportunityAwareRDWRedirector : Redirector
{
    private const float DefaultRotationCapDegreesPerSecond = 30f;
    private const float DefaultCurvatureCapDegreesPerSecond = 15f;

    [System.Serializable]
    private struct TemporalState
    {
        public Vector2 positionReal;
        public Vector2 forwardReal;
        public float speed;
        public float angularSpeed;
        public float nearestBoundaryDistance;
        public float predictedBoundaryDistance;
        public float leftClearance;
        public float rightClearance;
        public float distanceToCenter;
        public float centerBearing;
        public float distanceToWaypoint;
        public Vector3 previousAppliedGains;
        public bool inReset;
    }

    private struct PredictorOutput
    {
        public float opportunityScore;
        public float steerability;
        public Vector3 gainBudget;
        public bool criticalBoundaryRisk;
        public bool naturalTurningDetected;
        public bool decelerationDetected;
    }

    private struct GlobalSafeField
    {
        public Vector2 direction;
        public float strength;
    }

    private struct BaseControlProposal
    {
        public float curvatureDegrees;
        public float rotationDegrees;
        public float translationGain;
        public int desiredDirection;
        public Vector2 desiredFacingDirection;
    }

    [Header("时间输入 (Temporal Input)")]
    [Min(4)]
    public int historyLength = 12;

    [Min(0.5f)]
    public float lateralProbeDistance = 4f;

    [Min(0.05f)]
    public float movementThresholdMetersPerSecond = 0.1f;

    [Min(0.1f)]
    public float rotationThresholdDegreesPerSecond = 8f;

    [Header("机会预测器 (Opportunity Predictor)")]
    [Min(0f)]
    public float turnOpportunityLowDegreesPerSecond = 12f;

    [Min(0f)]
    public float turnOpportunityHighDegreesPerSecond = 45f;

    [Min(0f)]
    public float decelerationOpportunityLow = 0.05f;

    [Min(0f)]
    public float decelerationOpportunityHigh = 0.3f;

    [Range(0f, 1f)]
    public float minOpportunityAlpha = 0.65f;

    [Range(0f, 1f)]
    public float lowOpportunityThreshold = 0.25f;

    [Range(0f, 1f)]
    public float fallbackAlphaOnCriticalRisk = 1f;

    [Range(0f, 1f)]
    public float steeringEpsilon = 0.12f;

    [Header("安全设置 (Safety)")]
    [Min(0.1f)]
    public float criticalBoundaryDistance = 0.75f;

    [Min(0.2f)]
    public float comfortableBoundaryDistance = 2.0f;

    [Min(0f)]
    public float forwardPredictionDistance = 1.25f;

    [Range(0f, 1f)]
    public float predictionWeight = 0.45f;

    [Min(0f)]
    public float predictionLateralSpread = 0.35f;

    [Range(0f, 1f)]
    public float lateralEscapeBlendAtCriticalRisk = 0.5f;

    [Range(0f, 1f)]
    public float lateralEscapeBlendWhenSafe = 0.15f;

    [Header("任务引导 (Task Guidance)")]
    [Range(0f, 1f)]
    public float waypointBlendWhenSafe = 0.55f;

    [Range(0f, 1f)]
    public float waypointBlendAtCriticalRisk = 0.3f;

    [Range(0f, 1f)]
    public float centerBlendWhenSafe = 0.3f;

    [Range(0f, 1f)]
    public float centerBlendAtCriticalRisk = 0.2f;

    [Range(0f, 1f)]
    public float globalSafeBlendWhenSafe = 0.4f;

    [Range(0f, 1f)]
    public float globalSafeBlendAtCriticalRisk = 0.6f;

    [Min(0.01f)]
    public float globalSafeMinDistance = 0.15f;

    [Min(0.1f)]
    public float globalSafeStrengthHigh = 3.5f;

    [Header("增益调度 (Gain Scheduling)")]
    [Range(0f, 1f)]
    public float minBudgetFactor = 0.6f;

    [Range(0f, 1f)]
    public float boundaryRiskBudgetWeight = 0.55f;

    [Range(0f, 1f)]
    public float steerabilityBudgetWeight = 0.35f;

    [Range(0f, 1f)]
    public float opportunityBudgetWeight = 0.1f;

    [Range(0f, 1f)]
    public float gainSmoothingFactor = 0.45f;

    [Range(0f, 1f)]
    public float translationSmoothingFactor = 0.3f;

    [Range(0f, 1f)]
    public float highRiskOverrideThreshold = 0.6f;

    [Range(0f, 1f)]
    public float highRiskBudgetFloor = 0.85f;

    [Range(0f, 1f)]
    public float secondaryAngularGainRatio = 0.35f;

    [Header("转向稳定性 (Steering Hysteresis)")]
    [Min(0f)]
    public float steeringLockDuration = 0.75f;

    [Range(0f, 1f)]
    public float steeringSwitchConfidence = 0.3f;

    [Range(0f, 1f)]
    public float highRiskSteeringUnlockThreshold = 0.85f;

    [Header("Reset 恢复 (Reset Recovery)")]
    [Min(0f)]
    public float postResetBoostDuration = 0.75f;

    [Range(0f, 1f)]
    public float postResetGainRetention = 0.4f;

    [Range(0f, 1f)]
    public float postResetAlphaFloor = 0.85f;

    [Header("调试 (Debug)")]
    public bool enableRuntimeLogging = true;
    public bool verboseRuntimeLogging = false;
    [Min(1)]
    public int debugLogEveryNFrames = 45;

    [Header("运行时状态 (Runtime State)")]
    [SerializeField]
    private float lastOpportunityScore;
    [SerializeField]
    private float lastSteerability;
    [SerializeField]
    private Vector3 lastGainBudget;
    [SerializeField]
    private Vector3 lastBaseControlSuggestion;
    [SerializeField]
    private Vector3 lastFinalAppliedGains;
    [SerializeField]
    private float lastBoundaryDistance;
    [SerializeField]
    private float lastPredictedBoundaryDistance;
    [SerializeField]
    private float lastGlobalSafeStrength;
    [SerializeField]
    private int lastSteerDirection;
    [SerializeField]
    private bool lastUsedCriticalFallback;
    [SerializeField]
    private string lastDecisionSummary = string.Empty;

    private readonly List<TemporalState> stateHistory = new List<TemporalState>();
    private Vector3 previousAppliedGains = Vector3.zero;
    private int previousSteeringDirection = 1;
    private int lockedSteeringDirection = 1;
    private float steeringLockTimer;
    private int updateCounter;
    private float postResetBoostTimer;

    public override void InjectRedirection()
    {
        // Reset 会突然改变用户当前所处的局部几何关系，
        // 因此控制器会丢弃短时记忆，并基于 reset 之后的新状态重新建立内部状态。
        if (redirectionManager.ifJustEndReset)
        {
            ResetInternalState();
        }

        // Run the controller as a strict pipeline: observe -> predict -> schedule -> inject.
        // 将控制器按严格的流水线方式运行：观测 -> 预测 -> 调度 -> 注入。
        AppendTemporalState(BuildTemporalState());

        PredictorOutput predictor = PredictOpportunity(); 
        BaseControlProposal baseControl = ComputeBaseControlProposal();

        Vector3 scheduledGains = ScheduleGains(baseControl, predictor, out bool usedCriticalFallback, out int selectedSteeringDirection);
        ApplyScheduledGains(scheduledGains);
        PublishDebugState(predictor, baseControl, scheduledGains, usedCriticalFallback, selectedSteeringDirection);
    }

    // 重置内部状态，清除历史数据和累积的增益信息
    private void ResetInternalState()
    {
        stateHistory.Clear();
        previousAppliedGains *= postResetGainRetention;
        lockedSteeringDirection = previousSteeringDirection;
        steeringLockTimer = steeringLockDuration * 0.5f;
        postResetBoostTimer = postResetBoostDuration;
        updateCounter = 0;
    }

    // 构建当前帧的时间状态快照，包含位置、速度、边界距离等关键信息
    private TemporalState BuildTemporalState()
    {
        float deltaTime = Mathf.Max(redirectionManager.GetDeltaTime(), Utilities.eps);
        Vector2 currPosReal = Utilities.FlattenedPos2D(redirectionManager.currPosReal);
        Vector2 prevPosReal = Utilities.FlattenedPos2D(redirectionManager.prevPosReal);
        Vector2 currForwardReal = Utilities.FlattenedDir2D(redirectionManager.currDirReal);
        if (currForwardReal.sqrMagnitude <= Utilities.eps)
        {
            currForwardReal = Vector2.up;
        }

        Vector2 centerPoint = GetTrackingSpaceCentroid();
        Vector2 toCenter = centerPoint - currPosReal;
        float nearestBoundaryDistance = Utilities.GetNearestDistToObstacleAndTrackingSpace(
            globalConfiguration.obstaclePolygons,
            globalConfiguration.trackingSpacePoints,
            currPosReal);
        float predictedBoundaryDistance = EstimatePredictedBoundaryDistance(currPosReal, currForwardReal);

        // 这是启发式预测器使用的紧凑"状态"。
        // 它混合了运动线索、空间安全线索、任务线索以及控制器自身的前一个输出，
        // 大致遵循 PDF 设计。
        return new TemporalState
        {
            positionReal = currPosReal,
            forwardReal = currForwardReal,
            speed = (currPosReal - prevPosReal).magnitude / deltaTime,
            angularSpeed = Mathf.Abs(Utilities.GetSignedAngle(redirectionManager.prevDirReal, redirectionManager.currDirReal)) / deltaTime,
            nearestBoundaryDistance = nearestBoundaryDistance,
            predictedBoundaryDistance = predictedBoundaryDistance,
            leftClearance = EstimateDirectionalClearance(currPosReal, Utilities.RotateVector(currForwardReal, -90f)),
            rightClearance = EstimateDirectionalClearance(currPosReal, Utilities.RotateVector(currForwardReal, 90f)),
            distanceToCenter = toCenter.magnitude,
            centerBearing = toCenter.sqrMagnitude > Utilities.eps
                ? Mathf.Abs(Utilities.GetSignedAngle(Utilities.UnFlatten(currForwardReal), Utilities.UnFlatten(toCenter.normalized)))
                : 0f,
            distanceToWaypoint = GetDistanceToWaypoint(),
            previousAppliedGains = previousAppliedGains,
            inReset = redirectionManager.inReset
        };
    }

    // 将新的时间状态添加到历史记录中，保持固定长度的滑动窗口
    private void AppendTemporalState(TemporalState temporalState)
    {
        stateHistory.Add(temporalState);
        while (stateHistory.Count > historyLength)
        {
            stateHistory.RemoveAt(0);
        }
    }

    // 启发式机会预测器：估计当前时刻的重定向机会和增益预算
    private PredictorOutput PredictOpportunity()
    {
        TemporalState currentState = GetCurrentState();
        float averageAngularSpeed = GetAverageAngularSpeed();
        float averageSpeed = GetAverageSpeed();
        float deceleration = Mathf.Max(0f, averageSpeed - currentState.speed);

        // 在 v0.1 中机会评估故意保持简单：
        // 如果用户已经在自然转向或减速，
        // 控制器认为此时更容易隐藏重定向。
        float turnScore = NormalizeRange(averageAngularSpeed, turnOpportunityLowDegreesPerSecond, turnOpportunityHighDegreesPerSecond);
        float decelerationScore = NormalizeRange(deceleration, decelerationOpportunityLow, decelerationOpportunityHigh);
        float opportunityScore = Mathf.Clamp01(0.65f * turnScore + 0.35f * decelerationScore);

        // 左右可转向性从侧向自由空间不平衡估计。
        // 正的 clearanceBias 表示左侧比右侧更开阔。
        float clearanceDelta = currentState.leftClearance - currentState.rightClearance;
        float clearanceNormalizer = Mathf.Max(Mathf.Max(currentState.leftClearance, currentState.rightClearance), 0.1f);
        float clearanceBias = Mathf.Clamp(clearanceDelta / clearanceNormalizer, -1f, 1f);

        GlobalSafeField globalSafeField = ComputeGlobalSafeField(currentState);
        Vector2 desiredFacingDirection = ComputeDesiredFacingDirection(currentState, clearanceBias, globalSafeField);
        int desiredDirection = ComputeDesiredSteeringDirection(desiredFacingDirection);
        float steerabilityMagnitude = Mathf.Clamp01(Mathf.Max(Mathf.Abs(clearanceBias), globalSafeField.strength));

        float boundaryRisk = ComputeCombinedBoundaryRisk(currentState);
        float budgetFactor = Mathf.Clamp01(
            minBudgetFactor
            + boundaryRiskBudgetWeight * boundaryRisk
            + steerabilityBudgetWeight * steerabilityMagnitude
            + opportunityBudgetWeight * opportunityScore);

        // 预测器不直接输出增益。
        // 它只说明调度器本帧被允许使用多少增益。
        float deltaTime = Mathf.Max(redirectionManager.GetDeltaTime(), Utilities.eps);
        float curvatureBudget = DefaultCurvatureCapDegreesPerSecond * deltaTime * budgetFactor;
        float rotationBudget = DefaultRotationCapDegreesPerSecond * deltaTime * budgetFactor;
        float translationBudget = Mathf.Lerp(0f, -globalConfiguration.MIN_TRANS_GAIN, budgetFactor);

        return new PredictorOutput
        {
            opportunityScore = opportunityScore,
            steerability = desiredDirection * steerabilityMagnitude,
            gainBudget = new Vector3(curvatureBudget, rotationBudget, translationBudget),
            criticalBoundaryRisk = currentState.nearestBoundaryDistance <= criticalBoundaryDistance,
            naturalTurningDetected = turnScore > 0.5f,
            decelerationDetected = decelerationScore > 0.5f
        };
    }

    // 计算基础控制提议：在不考虑机会的情况下，确定应该施加的转向和增益
    private BaseControlProposal ComputeBaseControlProposal()
    {
        TemporalState currentState = GetCurrentState();
        float deltaTime = Mathf.Max(redirectionManager.GetDeltaTime(), Utilities.eps);
        float boundaryRisk = ComputeCombinedBoundaryRisk(currentState);
        float clearanceDelta = currentState.leftClearance - currentState.rightClearance;
        float clearanceNormalizer = Mathf.Max(Mathf.Max(currentState.leftClearance, currentState.rightClearance), 0.1f);
        float clearanceBias = Mathf.Clamp(clearanceDelta / clearanceNormalizer, -1f, 1f);

        GlobalSafeField globalSafeField = ComputeGlobalSafeField(currentState);
        Vector2 desiredFacingDirection = ComputeDesiredFacingDirection(currentState, clearanceBias, globalSafeField);
        int desiredDirection = ComputeDesiredSteeringDirection(desiredFacingDirection);
        if (desiredDirection == 0)
        {
            desiredDirection = previousSteeringDirection;
        }

        // 基础提议是控制器的几何部分：
        // "如果暂时忽略机会，现在应该向哪个方向以及多强地尝试引导这个用户？"
        float curvatureDegrees = 0f;
        if (currentState.speed > movementThresholdMetersPerSecond)
        {
            float rotationFromCurvature = Mathf.Rad2Deg * (Utilities.FlattenedPos2D(redirectionManager.currPosReal - redirectionManager.prevPosReal).magnitude / globalConfiguration.CURVATURE_RADIUS);
            float curvatureCap = DefaultCurvatureCapDegreesPerSecond * deltaTime;
            curvatureDegrees = desiredDirection * Mathf.Min(rotationFromCurvature, curvatureCap);
        }

        float rotationDegrees = 0f;
        if (currentState.angularSpeed >= rotationThresholdDegreesPerSecond)
        {
            float rotationCap = DefaultRotationCapDegreesPerSecond * deltaTime;
            float gain = redirectionManager.deltaDir * desiredDirection < 0
                ? Mathf.Abs(redirectionManager.deltaDir * globalConfiguration.MIN_ROT_GAIN)
                : Mathf.Abs(redirectionManager.deltaDir * globalConfiguration.MAX_ROT_GAIN);
            rotationDegrees = desiredDirection * Mathf.Min(gain, rotationCap);
        }

        float translationGain = 0f;
        if (currentState.speed > movementThresholdMetersPerSecond && boundaryRisk > 0.35f)
        {
            translationGain = Mathf.Lerp(0f, -globalConfiguration.MIN_TRANS_GAIN, boundaryRisk);
        }

        return new BaseControlProposal
        {
            curvatureDegrees = curvatureDegrees,
            rotationDegrees = rotationDegrees,
            translationGain = translationGain,
            desiredDirection = desiredDirection,
            desiredFacingDirection = desiredFacingDirection
        };
    }

    // 增益调度器：根据机会评估和预算约束，调整并平滑最终应用的增益
    private Vector3 ScheduleGains(BaseControlProposal baseControl, PredictorOutput predictor, out bool usedCriticalFallback, out int selectedSteeringDirection)
    {
        // 机会调节强度，而不是方向。
        // 即使机会很弱，我们仍然允许非零的最小 alpha，
        // 这样控制器可以响应而不是完全空闲。
        float effectiveAlpha = Mathf.Lerp(minOpportunityAlpha, 1f, predictor.opportunityScore);
        usedCriticalFallback = predictor.criticalBoundaryRisk && predictor.opportunityScore < lowOpportunityThreshold;
        if (usedCriticalFallback)
        {
            // 如果边界风险紧迫，安全性优先于隐蔽性。
            effectiveAlpha = Mathf.Max(effectiveAlpha, fallbackAlphaOnCriticalRisk);
        }

        // 当预测器的可转向性足够自信时，优先使用其符号；
        // 否则回退到基础控制器的几何方向。
        selectedSteeringDirection = Mathf.Abs(predictor.steerability) >= steeringEpsilon
            ? (int)Mathf.Sign(predictor.steerability)
            : baseControl.desiredDirection;
        if (selectedSteeringDirection == 0)
        {
            selectedSteeringDirection = previousSteeringDirection;
        }

        float deltaTime = Mathf.Max(redirectionManager.GetDeltaTime(), Utilities.eps);
        float boundaryRisk = ComputeCombinedBoundaryRisk(GetCurrentState());
        bool highRiskOverride = boundaryRisk >= highRiskOverrideThreshold;
        if (highRiskOverride)
        {
            effectiveAlpha = Mathf.Max(effectiveAlpha, 0.9f);
        }

        if (postResetBoostTimer > 0f)
        {
            effectiveAlpha = Mathf.Max(effectiveAlpha, postResetAlphaFloor);
            postResetBoostTimer = Mathf.Max(0f, postResetBoostTimer - deltaTime);
        }

        selectedSteeringDirection = ApplySteeringHysteresis(
            selectedSteeringDirection,
            Mathf.Abs(predictor.steerability),
            boundaryRisk,
            deltaTime);

        float curvatureBudget = predictor.gainBudget.x;
        float rotationBudget = predictor.gainBudget.y;
        if (highRiskOverride)
        {
            curvatureBudget = Mathf.Max(curvatureBudget, DefaultCurvatureCapDegreesPerSecond * deltaTime * highRiskBudgetFloor);
            rotationBudget = Mathf.Max(rotationBudget, DefaultRotationCapDegreesPerSecond * deltaTime * highRiskBudgetFloor);
        }

        // 首先应用机会缩放，然后用预测的预算进行裁剪。
        float targetCurvature = selectedSteeringDirection * Mathf.Min(Mathf.Abs(baseControl.curvatureDegrees) * effectiveAlpha, curvatureBudget);
        float targetRotation = selectedSteeringDirection * Mathf.Min(Mathf.Abs(baseControl.rotationDegrees) * effectiveAlpha, rotationBudget);
        float targetTranslation = Mathf.Min(Mathf.Abs(baseControl.translationGain) * effectiveAlpha, predictor.gainBudget.z);

        // 平滑调度的增益，使控制器不会在帧之间抖动。
        float smoothedCurvature = Mathf.Lerp(previousAppliedGains.x, targetCurvature, gainSmoothingFactor);
        float smoothedRotation = Mathf.Lerp(previousAppliedGains.y, targetRotation, gainSmoothingFactor);
        float smoothedTranslation = Mathf.Lerp(previousAppliedGains.z, targetTranslation, translationSmoothingFactor);

        return new Vector3(smoothedCurvature, smoothedRotation, smoothedTranslation);
    }

    // 应用调度的增益：使用 OpenRDW 的现有注入接口
    private void ApplyScheduledGains(Vector3 scheduledGains)
    {
        bool rotationDominant = Mathf.Abs(scheduledGains.y) >= Mathf.Abs(scheduledGains.x);
        float appliedCurvature = scheduledGains.x * (rotationDominant ? secondaryAngularGainRatio : 1f);
        float appliedRotation = scheduledGains.y * (rotationDominant ? 1f : secondaryAngularGainRatio);
        float appliedTranslation = scheduledGains.z;

        if (appliedTranslation > 0f && redirectionManager.deltaPos.sqrMagnitude > Utilities.eps)
        {
            InjectTranslation(appliedTranslation * redirectionManager.deltaPos);
        }

        if (!Mathf.Approximately(appliedRotation, 0f))
        {
            InjectRotation(appliedRotation);
        }

        if (!Mathf.Approximately(appliedCurvature, 0f))
        {
            InjectCurvature(appliedCurvature);
        }

        previousAppliedGains = new Vector3(appliedCurvature, appliedRotation, appliedTranslation);
    }

    private void PublishDebugState(
        PredictorOutput predictor,
        BaseControlProposal baseControl,
        Vector3 scheduledGains,
        bool usedCriticalFallback,
        int selectedSteeringDirection)
    {
        updateCounter++;
        previousSteeringDirection = selectedSteeringDirection;

        TemporalState currentState = GetCurrentState();
        lastOpportunityScore = predictor.opportunityScore;
        lastSteerability = predictor.steerability;
        lastGainBudget = predictor.gainBudget;
        lastBaseControlSuggestion = new Vector3(baseControl.curvatureDegrees, baseControl.rotationDegrees, baseControl.translationGain);
        lastFinalAppliedGains = previousAppliedGains;
        lastBoundaryDistance = currentState.nearestBoundaryDistance;
        lastPredictedBoundaryDistance = currentState.predictedBoundaryDistance;
        lastGlobalSafeStrength = ComputeGlobalSafeField(currentState).strength;
        lastSteerDirection = selectedSteeringDirection;
        lastUsedCriticalFallback = usedCriticalFallback;
        lastDecisionSummary = string.Format(
            "O={0:F2}, steer={1:F2}, boundary=({2:F2}->{3:F2}), globalSafe={4:F2}, budget=({5:F2},{6:F2},{7:F2}), base=({8:F2},{9:F2},{10:F2}), final=({11:F2},{12:F2},{13:F2}), naturalTurn={14}, decel={15}, criticalFallback={16}, postResetBoost={17:F2}",
            predictor.opportunityScore,
            predictor.steerability,
            currentState.nearestBoundaryDistance,
            currentState.predictedBoundaryDistance,
            lastGlobalSafeStrength,
            predictor.gainBudget.x,
            predictor.gainBudget.y,
            predictor.gainBudget.z,
            baseControl.curvatureDegrees,
            baseControl.rotationDegrees,
            baseControl.translationGain,
            previousAppliedGains.x,
            previousAppliedGains.y,
            previousAppliedGains.z,
            predictor.naturalTurningDetected,
            predictor.decelerationDetected,
            usedCriticalFallback,
            postResetBoostTimer);

        if (enableRuntimeLogging && (verboseRuntimeLogging || updateCounter % debugLogEveryNFrames == 0))
        {
            Debug.Log("[OpportunityAwareRDW] " + lastDecisionSummary, this);
        }
    }

    // 获取当前状态快照
    private TemporalState GetCurrentState()
    {
        if (stateHistory.Count == 0)
        {
            return BuildTemporalState();
        }

        return stateHistory[stateHistory.Count - 1];
    }

    // 计算历史窗口内的平均速度
    private float GetAverageSpeed()
    {
        if (stateHistory.Count == 0)
        {
            return 0f;
        }

        float sum = 0f;
        for (int i = 0; i < stateHistory.Count; i++)
        {
            sum += stateHistory[i].speed;
        }
        return sum / stateHistory.Count;
    }

    // 计算历史窗口内的平均角速度
    private float GetAverageAngularSpeed()
    {
        if (stateHistory.Count == 0)
        {
            return 0f;
        }

        float sum = 0f;
        for (int i = 0; i < stateHistory.Count; i++)
        {
            sum += stateHistory[i].angularSpeed;
        }
        return sum / stateHistory.Count;
    }

    // 计算到目标路点的距离
    private float GetDistanceToWaypoint()
    {
        if (redirectionManager.targetWaypoint == null)
        {
            return 0f;
        }

        return Vector2.Distance(
            Utilities.FlattenedPos2D(redirectionManager.currPos),
            Utilities.FlattenedPos2D(redirectionManager.targetWaypoint.position));
    }

    // 计算追踪空间质心
    private Vector2 GetTrackingSpaceCentroid()
    {
        if (globalConfiguration.trackingSpacePoints == null || globalConfiguration.trackingSpacePoints.Count == 0)
        {
            return Utilities.FlattenedPos2D(redirectionManager.trackingSpace.position);
        }

        Vector2 sum = Vector2.zero;
        for (int i = 0; i < globalConfiguration.trackingSpacePoints.Count; i++)
        {
            sum += globalConfiguration.trackingSpacePoints[i];
        }
        return sum / globalConfiguration.trackingSpacePoints.Count;
    }

    // 计算边界风险：距离越近，风险越高（返回 0-1 之间的值）
    private float ComputeBoundaryRisk(float boundaryDistance)
    {
        if (comfortableBoundaryDistance <= criticalBoundaryDistance)
        {
            return boundaryDistance <= criticalBoundaryDistance ? 1f : 0f;
        }

        return 1f - Mathf.Clamp01(Mathf.InverseLerp(criticalBoundaryDistance, comfortableBoundaryDistance, boundaryDistance));
    }

    public Vector2 GetRecommendedResetDirection()
    {
        TemporalState currentState = GetCurrentState();
        Vector2 desiredFacingDirection = ComputeResetFacingDirection(currentState);
        if (desiredFacingDirection.sqrMagnitude <= Utilities.eps)
        {
            return currentState.forwardReal;
        }

        return desiredFacingDirection.normalized;
    }

    public float GetRecommendedResetRotationDegrees()
    {
        TemporalState currentState = GetCurrentState();
        Vector2 currentDirection = currentState.forwardReal.sqrMagnitude > Utilities.eps
            ? currentState.forwardReal.normalized
            : Utilities.FlattenedDir2D(redirectionManager.currDirReal);
        Vector2 targetDirection = GetRecommendedResetDirection();
        float targetAngle = Vector2.Angle(currentDirection, targetDirection);
        float boundaryRisk = ComputeCombinedBoundaryRisk(currentState);

        float minimumResetAngle = Mathf.Lerp(75f, 120f, boundaryRisk);
        return Mathf.Clamp(Mathf.Max(targetAngle, minimumResetAngle), 75f, 180f);
    }

    // 计算期望朝向方向：结合追踪空间中心引导和侧向逃逸偏置
    private Vector2 ComputeDesiredFacingDirection(TemporalState currentState, float clearanceBias, GlobalSafeField globalSafeField)
    {
        // 控制器混合四种直觉：
        // 1) 跟随当前 waypoint 的任务方向
        // 2) 跟随 APF 风格的全局安全方向
        // 3) 转向回到追踪空间中心
        // 4) 偏向有更多侧向自由空间的一侧
        Vector2 waypointDirection = GetWaypointDirectionReal(currentState);
        Vector2 globalSafeDirection = globalSafeField.direction;
        Vector2 centerDirection = GetTrackingSpaceCentroid() - currentState.positionReal;
        if (centerDirection.sqrMagnitude <= Utilities.eps)
        {
            centerDirection = currentState.forwardReal;
        }
        centerDirection.Normalize();

        Vector2 saferLateralDirection = clearanceBias >= 0f
            ? Utilities.RotateVector(currentState.forwardReal, -90f)
            : Utilities.RotateVector(currentState.forwardReal, 90f);
        saferLateralDirection.Normalize();

        float boundaryRisk = ComputeCombinedBoundaryRisk(currentState);
        float waypointBlend = Mathf.Lerp(waypointBlendWhenSafe, waypointBlendAtCriticalRisk, boundaryRisk);
        float globalSafeBlend = Mathf.Lerp(globalSafeBlendWhenSafe, globalSafeBlendAtCriticalRisk, boundaryRisk);
        float centerBlend = Mathf.Lerp(centerBlendWhenSafe, centerBlendAtCriticalRisk, boundaryRisk);
        float lateralBlend = Mathf.Lerp(lateralEscapeBlendWhenSafe, lateralEscapeBlendAtCriticalRisk, boundaryRisk);
        lateralBlend *= Mathf.Clamp01(Mathf.Abs(clearanceBias));

        if (globalSafeField.strength <= Utilities.eps)
        {
            globalSafeBlend = 0f;
        }

        float totalBlend = waypointBlend + globalSafeBlend + centerBlend + lateralBlend;
        Vector2 desiredFacingDirection =
            waypointBlend * waypointDirection +
            globalSafeBlend * globalSafeDirection +
            centerBlend * centerDirection +
            lateralBlend * saferLateralDirection;
        if (totalBlend > Utilities.eps)
        {
            desiredFacingDirection /= totalBlend;
        }
        desiredFacingDirection.Normalize();
        if (desiredFacingDirection.sqrMagnitude <= Utilities.eps)
        {
            desiredFacingDirection = waypointDirection;
        }
        return desiredFacingDirection;
    }

    private Vector2 ComputeResetFacingDirection(TemporalState currentState)
    {
        GlobalSafeField globalSafeField = ComputeGlobalSafeField(currentState);
        Vector2 centerDirection = GetTrackingSpaceCentroid() - currentState.positionReal;
        if (centerDirection.sqrMagnitude <= Utilities.eps)
        {
            centerDirection = currentState.forwardReal;
        }
        centerDirection.Normalize();

        Vector2 awayFromBoundaryDirection = GetAwayFromNearestBoundaryDirection(currentState.positionReal);
        if (awayFromBoundaryDirection.sqrMagnitude <= Utilities.eps)
        {
            awayFromBoundaryDirection = centerDirection;
        }
        awayFromBoundaryDirection.Normalize();

        Vector2 globalSafeDirection = globalSafeField.direction.sqrMagnitude > Utilities.eps
            ? globalSafeField.direction.normalized
            : awayFromBoundaryDirection;

        Vector2 desiredFacingDirection =
            0.6f * globalSafeDirection +
            0.3f * awayFromBoundaryDirection +
            0.1f * centerDirection;

        if (desiredFacingDirection.sqrMagnitude <= Utilities.eps)
        {
            desiredFacingDirection = centerDirection;
        }

        return desiredFacingDirection.normalized;
    }

    private Vector2 GetWaypointDirectionReal(TemporalState currentState)
    {
        if (redirectionManager.targetWaypoint == null)
        {
            return currentState.forwardReal;
        }

        Vector3 waypointDirectionVirtual = Utilities.FlattenedPos3D(redirectionManager.targetWaypoint.position - redirectionManager.currPos);
        if (waypointDirectionVirtual.sqrMagnitude <= Utilities.eps)
        {
            return currentState.forwardReal;
        }

        Vector2 waypointDirectionReal = Utilities.FlattenedDir2D(redirectionManager.GetDirReal(waypointDirectionVirtual));
        if (waypointDirectionReal.sqrMagnitude <= Utilities.eps)
        {
            return currentState.forwardReal;
        }

        return waypointDirectionReal.normalized;
    }

    private GlobalSafeField ComputeGlobalSafeField(TemporalState currentState)
    {
        Vector2 currPosReal = currentState.positionReal;
        Vector2 accumulatedForce = Vector2.zero;

        for (int i = 0; i < globalConfiguration.trackingSpacePoints.Count; i++)
        {
            Vector2 p = globalConfiguration.trackingSpacePoints[i];
            Vector2 q = globalConfiguration.trackingSpacePoints[(i + 1) % globalConfiguration.trackingSpacePoints.Count];
            Vector2 nearestPos = Utilities.GetNearestPos(currPosReal, new List<Vector2> { p, q });
            accumulatedForce += GetRepulsiveContribution(currPosReal, nearestPos);
        }

        foreach (var obstaclePolygon in globalConfiguration.obstaclePolygons)
        {
            Vector2 nearestPos = Utilities.GetNearestPos(currPosReal, obstaclePolygon);
            accumulatedForce += GetRepulsiveContribution(currPosReal, nearestPos);
        }

        foreach (var user in globalConfiguration.redirectedAvatars)
        {
            var rm = user.GetComponent<RedirectionManager>();
            if (rm == null || rm == redirectionManager)
            {
                continue;
            }

            Vector2 otherPos = Utilities.FlattenedPos2D(rm.currPosReal);
            accumulatedForce += 0.35f * GetRepulsiveContribution(currPosReal, otherPos);
        }

        if (accumulatedForce.sqrMagnitude <= Utilities.eps)
        {
            return new GlobalSafeField
            {
                direction = currentState.forwardReal,
                strength = 0f
            };
        }

        return new GlobalSafeField
        {
            direction = accumulatedForce.normalized,
            strength = NormalizeRange(accumulatedForce.magnitude, 0f, globalSafeStrengthHigh)
        };
    }

    private Vector2 GetRepulsiveContribution(Vector2 currPosReal, Vector2 nearestPos)
    {
        Vector2 delta = currPosReal - nearestPos;
        float distance = Mathf.Max(delta.magnitude, globalSafeMinDistance);
        if (distance <= Utilities.eps)
        {
            return Vector2.zero;
        }

        return delta.normalized / (distance * distance);
    }

    private Vector2 GetAwayFromNearestBoundaryDirection(Vector2 currentPositionReal)
    {
        float nearestDistance = float.MaxValue;
        Vector2 nearestPoint = Vector2.zero;
        bool foundNearest = false;

        for (int i = 0; i < globalConfiguration.trackingSpacePoints.Count; i++)
        {
            Vector2 p = globalConfiguration.trackingSpacePoints[i];
            Vector2 q = globalConfiguration.trackingSpacePoints[(i + 1) % globalConfiguration.trackingSpacePoints.Count];
            Vector2 candidatePoint = Utilities.GetNearestPos(currentPositionReal, new List<Vector2> { p, q });
            float candidateDistance = Vector2.Distance(currentPositionReal, candidatePoint);
            if (candidateDistance < nearestDistance)
            {
                nearestDistance = candidateDistance;
                nearestPoint = candidatePoint;
                foundNearest = true;
            }
        }

        foreach (var obstaclePolygon in globalConfiguration.obstaclePolygons)
        {
            Vector2 candidatePoint = Utilities.GetNearestPos(currentPositionReal, obstaclePolygon);
            float candidateDistance = Vector2.Distance(currentPositionReal, candidatePoint);
            if (candidateDistance < nearestDistance)
            {
                nearestDistance = candidateDistance;
                nearestPoint = candidatePoint;
                foundNearest = true;
            }
        }

        if (!foundNearest)
        {
            return Vector2.zero;
        }

        return currentPositionReal - nearestPoint;
    }

    private int ApplySteeringHysteresis(int candidateDirection, float steerabilityConfidence, float boundaryRisk, float deltaTime)
    {
        if (candidateDirection == 0)
        {
            candidateDirection = lockedSteeringDirection != 0 ? lockedSteeringDirection : previousSteeringDirection;
        }

        steeringLockTimer = Mathf.Max(0f, steeringLockTimer - deltaTime);

        bool sameDirection = candidateDirection == lockedSteeringDirection;
        bool highRiskUnlock = boundaryRisk >= highRiskSteeringUnlockThreshold;
        bool strongEnoughToSwitch = steerabilityConfidence >= steeringSwitchConfidence;

        if (sameDirection)
        {
            if (steerabilityConfidence >= steeringEpsilon)
            {
                steeringLockTimer = Mathf.Max(steeringLockTimer, steeringLockDuration * 0.5f);
            }
            return lockedSteeringDirection;
        }

        if (steeringLockTimer > 0f && !highRiskUnlock && !strongEnoughToSwitch)
        {
            return lockedSteeringDirection;
        }

        lockedSteeringDirection = candidateDirection;
        steeringLockTimer = steeringLockDuration;
        return lockedSteeringDirection;
    }

    // 计算期望转向方向（左转或右转）
    private int ComputeDesiredSteeringDirection(Vector2 desiredFacingDirection)
    {
        if (desiredFacingDirection.sqrMagnitude <= Utilities.eps)
        {
            return 0;
        }

        float signedAngle = Utilities.GetSignedAngle(
            Utilities.UnFlatten(Utilities.FlattenedDir2D(redirectionManager.currDirReal)),
            Utilities.UnFlatten(desiredFacingDirection.normalized));

        // OpenRDW 转向符号使用"反向转向以使用户反向进入期望的物理方向"的约定，
        // 与 SteerTo/APF 重定向器匹配。
        int desiredSteeringDirection = -(int)Mathf.Sign(signedAngle);
        if (desiredSteeringDirection == 0)
        {
            desiredSteeringDirection = previousSteeringDirection;
        }
        return desiredSteeringDirection;
    }

    private float ComputeCombinedBoundaryRisk(TemporalState currentState)
    {
        float instantRisk = ComputeBoundaryRisk(currentState.nearestBoundaryDistance);
        float predictedRisk = ComputeBoundaryRisk(currentState.predictedBoundaryDistance);
        return Mathf.Max(instantRisk, Mathf.Lerp(instantRisk, predictedRisk, predictionWeight));
    }

    // 估计指定方向的间隙距离（用于左右间隙检测）
    private float EstimateDirectionalClearance(Vector2 origin, Vector2 direction)
    {
        if (direction.sqrMagnitude <= Utilities.eps)
        {
            return lateralProbeDistance;
        }

        // 用短射线横向探测，并取与追踪空间边缘或障碍物边缘的最近命中。
        // 这提供了"这一侧有多少空间？"的廉价度量。
        Vector2 rayDirection = direction.normalized;
        float minDistance = lateralProbeDistance;

        for (int i = 0; i < globalConfiguration.trackingSpacePoints.Count; i++)
        {
            Vector2 start = globalConfiguration.trackingSpacePoints[i];
            Vector2 end = globalConfiguration.trackingSpacePoints[(i + 1) % globalConfiguration.trackingSpacePoints.Count];
            minDistance = Mathf.Min(minDistance, RaySegmentDistance(origin, rayDirection, start, end, lateralProbeDistance));
        }

        for (int polygonIndex = 0; polygonIndex < globalConfiguration.obstaclePolygons.Count; polygonIndex++)
        {
            List<Vector2> polygon = globalConfiguration.obstaclePolygons[polygonIndex];
            for (int i = 0; i < polygon.Count; i++)
            {
                Vector2 start = polygon[i];
                Vector2 end = polygon[(i + 1) % polygon.Count];
                minDistance = Mathf.Min(minDistance, RaySegmentDistance(origin, rayDirection, start, end, lateralProbeDistance));
            }
        }

        return minDistance;
    }

    private float EstimatePredictedBoundaryDistance(Vector2 origin, Vector2 forward)
    {
        if (forwardPredictionDistance <= Utilities.eps || forward.sqrMagnitude <= Utilities.eps)
        {
            return Utilities.GetNearestDistToObstacleAndTrackingSpace(
                globalConfiguration.obstaclePolygons,
                globalConfiguration.trackingSpacePoints,
                origin);
        }

        Vector2 normalizedForward = forward.normalized;
        Vector2 right = Utilities.RotateVector(normalizedForward, 90f);

        Vector2 forwardPoint = origin + normalizedForward * forwardPredictionDistance;
        Vector2 forwardLeftPoint = forwardPoint - right * predictionLateralSpread;
        Vector2 forwardRightPoint = forwardPoint + right * predictionLateralSpread;

        float forwardDistance = Utilities.GetNearestDistToObstacleAndTrackingSpace(
            globalConfiguration.obstaclePolygons,
            globalConfiguration.trackingSpacePoints,
            forwardPoint);
        float forwardLeftDistance = Utilities.GetNearestDistToObstacleAndTrackingSpace(
            globalConfiguration.obstaclePolygons,
            globalConfiguration.trackingSpacePoints,
            forwardLeftPoint);
        float forwardRightDistance = Utilities.GetNearestDistToObstacleAndTrackingSpace(
            globalConfiguration.obstaclePolygons,
            globalConfiguration.trackingSpacePoints,
            forwardRightPoint);

        return Mathf.Min(forwardDistance, Mathf.Min(forwardLeftDistance, forwardRightDistance));
    }

    // 计算射线与线段之间的距离（用于间隙检测）
    private float RaySegmentDistance(Vector2 rayOrigin, Vector2 rayDirection, Vector2 segmentStart, Vector2 segmentEnd, float fallbackDistance)
    {
        Vector2 segmentDirection = segmentEnd - segmentStart;
        float denominator = Utilities.Cross(rayDirection, segmentDirection);
        if (Mathf.Abs(denominator) <= Utilities.eps)
        {
            return fallbackDistance;
        }

        Vector2 startDelta = segmentStart - rayOrigin;
        float rayDistance = Utilities.Cross(startDelta, segmentDirection) / denominator;
        float segmentRatio = Utilities.Cross(startDelta, rayDirection) / denominator;
        if (rayDistance < 0f || segmentRatio < 0f || segmentRatio > 1f)
        {
            return fallbackDistance;
        }

        return rayDistance;
    }

    // 将值归一化到 [0, 1] 范围
    private float NormalizeRange(float value, float low, float high)
    {
        if (high <= low)
        {
            return value > low ? 1f : 0f;
        }

        return Mathf.Clamp01((value - low) / (high - low));
    }
}
