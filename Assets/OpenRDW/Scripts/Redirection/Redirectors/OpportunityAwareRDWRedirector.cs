using UnityEngine;
using System.Collections.Generic;

// Opportunity-Aware RDW Controller 当前实现：
// 1）底层使用 APF 风格全局安全方向作为 subtle redirection 的方向骨架
// 2）机会层只估计“当前是否更适合隐藏增益”，不再覆盖 APF 方向
// 3）调度层只做轻量缩放与上限保护
// 4）高风险时直接退回纯 APF 输出
public class OpportunityAwareRDWRedirector : APF_Redirector
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
        public int directionalConsistency;
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

    private struct ThomasApfBackbone
    {
        public Vector2 negativeGradient;
        public float repulsiveForce;
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
    public float lowOpportunityThreshold = 0.25f;

    [Range(0f, 1f)]
    public float steeringEpsilon = 0.12f;

    [Header("路径阶段机会 (Path-Stage Opportunity)")]
    [Min(0f)]
    public float waypointOpportunityNearDistance = 1.5f;

    [Min(0f)]
    public float waypointTurnLowDegrees = 20f;

    [Min(0f)]
    public float waypointTurnHighDegrees = 75f;

    [Range(0f, 1f)]
    public float behaviorOpportunityWeight = 0.25f;

    [Range(0f, 1f)]
    public float waypointTurnOpportunityWeight = 0.35f;

    [Range(0f, 1f)]
    public float riskTrendOpportunityWeight = 0.2f;

    [Range(0f, 1f)]
    public float apfChangeOpportunityWeight = 0.1f;

    [Range(0f, 1f)]
    public float timePressureOpportunityWeight = 0.1f;

    [Header("安全设置 (Safety)")]
    [Min(0.1f)]
    public float criticalBoundaryDistance = 0.75f;

    [Min(0.2f)]
    public float comfortableBoundaryDistance = 2.0f;

    [Min(0f)]
    public float boundaryRiskRiseLow = 0.03f;

    [Min(0f)]
    public float boundaryRiskRiseHigh = 0.18f;

    [Min(0f)]
    public float forwardPredictionDistance = 1.25f;

    [Range(0f, 1f)]
    public float predictionWeight = 0.45f;

    [Min(0f)]
    public float predictionLateralSpread = 0.35f;

    [Min(0f)]
    public float apfDirectionChangeLowDegrees = 5f;

    [Min(0f)]
    public float apfDirectionChangeHighDegrees = 25f;

    [Min(0f)]
    public float timeToBoundaryLowSeconds = 0.75f;

    [Min(0f)]
    public float timeToBoundaryHighSeconds = 2.5f;

    [Header("APF 安全骨架 (APF Backbone)")]

    [Min(0.01f)]
    public float globalSafeMinDistance = 0.15f;

    [Min(0.1f)]
    public float globalSafeStrengthHigh = 3.5f;

    [Header("增益调度 (Gain Scheduling)")]
    [Range(0f, 1f)]
    public float minBudgetFactor = 0.95f;

    [Range(0f, 1f)]
    public float boundaryRiskBudgetWeight = 0.05f;

    [Range(0f, 1f)]
    public float steerabilityBudgetWeight = 0.0f;

    [Range(0f, 1f)]
    public float opportunityBudgetWeight = 0.0f;

    [Range(0f, 1f)]
    public float gainSmoothingFactor = 0.45f;

    [Range(0f, 1f)]
    public float translationSmoothingFactor = 0.3f;

    [Range(0f, 1f)]
    public float highRiskOverrideThreshold = 0.6f;

    [Range(0f, 1f)]
    public float secondaryAngularGainRatio = 0.35f;

    [Header("机会软调制 (Soft Opportunity Modulation)")]
    [Range(1f, 1.05f)]
    public float lowOpportunityConflictBeta = 1f;

    [Range(1f, 1.05f)]
    public float lowOpportunityNeutralBeta = 1f;

    [Range(1f, 1.05f)]
    public float neutralOpportunityBeta = 1f;

    [Range(1f, 1.1f)]
    public float highOpportunityNeutralBeta = 1.05f;

    [Range(1f, 1.2f)]
    public float highOpportunityAlignedBeta = 1.1f;

    [Range(1f, 1.5f)]
    public float baseBudgetCapScale = 1.15f;

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
    private Vector3 lastBonusGains;
    [SerializeField]
    private Vector3 lastFinalAppliedGains;
    [SerializeField]
    private float lastBoundaryDistance;
    [SerializeField]
    private float lastPredictedBoundaryDistance;
    [SerializeField]
    private float lastGlobalSafeStrength;
    [SerializeField]
    private int lastDirectionalConsistency;
    [SerializeField]
    private int lastSteerDirection;
    [SerializeField]
    private bool lastUsedCriticalFallback;
    [SerializeField]
    private string lastDecisionSummary = string.Empty;

    private readonly List<TemporalState> stateHistory = new List<TemporalState>();
    private Vector3 previousAppliedGains = Vector3.zero;
    private Vector3 previousBonusGains = Vector3.zero;
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
        UpdateTotalForcePointer(baseControl.desiredFacingDirection.sqrMagnitude > Utilities.eps
            ? baseControl.desiredFacingDirection.normalized
            : Vector2.zero);

        Vector3 scheduledGains = ScheduleGains(baseControl, predictor, out bool usedCriticalFallback, out int selectedSteeringDirection);
        ApplyScheduledGains(scheduledGains);
        PublishDebugState(predictor, baseControl, scheduledGains, usedCriticalFallback, selectedSteeringDirection);
    }

    // 重置内部状态，清除历史数据和累积的增益信息
    private void ResetInternalState()
    {
        stateHistory.Clear();
        previousAppliedGains *= postResetGainRetention;
        previousBonusGains *= postResetGainRetention;
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
        float behaviorOpportunity = Mathf.Clamp01(0.65f * turnScore + 0.35f * decelerationScore);

        float waypointProximityScore = ComputeWaypointProximityScore(currentState.distanceToWaypoint);
        float upcomingTurnScore = ComputeUpcomingTurnScore(currentState);
        float waypointTurnOpportunity = waypointProximityScore * upcomingTurnScore;

        float boundaryRiskTrendScore = ComputeBoundaryRiskTrendScore();
        float apfDirectionChangeScore = ComputeApfDirectionChangeScore(currentState);
        float timePressureScore = ComputeTimePressureScore(currentState);

        float opportunityScore = Mathf.Clamp01(
            behaviorOpportunityWeight * behaviorOpportunity +
            waypointTurnOpportunityWeight * waypointTurnOpportunity +
            riskTrendOpportunityWeight * boundaryRiskTrendScore +
            apfChangeOpportunityWeight * apfDirectionChangeScore +
            timePressureOpportunityWeight * timePressureScore);

        ThomasApfBackbone apfBackbone = ComputeThomasApfBackbone(currentState);
        Vector2 desiredFacingDirection = ComputeDesiredFacingDirection(currentState, apfBackbone.negativeGradient);
        float steerabilityMagnitude = Mathf.Clamp01(ComputeGlobalSafeField(currentState).strength);
        int directionalConsistency = ComputeDirectionalConsistency(desiredFacingDirection, averageAngularSpeed);

        float boundaryRisk = ComputeCombinedBoundaryRisk(currentState);
        float budgetFactor = Mathf.Clamp01(
            minBudgetFactor
            + boundaryRiskBudgetWeight * boundaryRisk
            + steerabilityBudgetWeight * steerabilityMagnitude
            + opportunityBudgetWeight * opportunityScore);

        float deltaTime = Mathf.Max(redirectionManager.GetDeltaTime(), Utilities.eps);
        float curvatureBudget = DefaultCurvatureCapDegreesPerSecond * deltaTime * (baseBudgetCapScale * budgetFactor);
        float rotationBudget = DefaultRotationCapDegreesPerSecond * deltaTime * (baseBudgetCapScale * budgetFactor);
        float translationBudget = Mathf.Lerp(0f, -globalConfiguration.MIN_TRANS_GAIN * baseBudgetCapScale, budgetFactor);

        return new PredictorOutput
        {
            opportunityScore = opportunityScore,
            steerability = steerabilityMagnitude,
            directionalConsistency = directionalConsistency,
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
        ThomasApfBackbone apfBackbone = ComputeThomasApfBackbone(currentState);
        Vector2 desiredFacingDirection = ComputeDesiredFacingDirection(currentState, apfBackbone.negativeGradient);
        int desiredDirection = ComputeDesiredSteeringDirection(desiredFacingDirection);
        if (desiredDirection == 0)
        {
            desiredDirection = previousSteeringDirection;
        }

        float curvatureDegrees = 0f;
        if (currentState.speed > movementThresholdMetersPerSecond)
        {
            float rotationFromCurvature = Mathf.Rad2Deg * (redirectionManager.deltaPos.magnitude / globalConfiguration.CURVATURE_RADIUS);
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
        if (currentState.speed > movementThresholdMetersPerSecond && Vector2.Dot(desiredFacingDirection, currentState.forwardReal) < 0f)
        {
            translationGain = -globalConfiguration.MIN_TRANS_GAIN;
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
        float deltaTime = Mathf.Max(redirectionManager.GetDeltaTime(), Utilities.eps);
        float boundaryRisk = ComputeCombinedBoundaryRisk(GetCurrentState());
        bool pureApfFallback = predictor.criticalBoundaryRisk || boundaryRisk >= highRiskOverrideThreshold;
        usedCriticalFallback = pureApfFallback;

        float effectiveAlpha = pureApfFallback ? 1f : GetSoftModulationBeta(predictor);
        if (postResetBoostTimer > 0f)
        {
            effectiveAlpha = pureApfFallback ? 1f : Mathf.Max(effectiveAlpha, postResetAlphaFloor);
            postResetBoostTimer = Mathf.Max(0f, postResetBoostTimer - deltaTime);
        }

        selectedSteeringDirection = baseControl.desiredDirection;
        if (selectedSteeringDirection == 0)
        {
            selectedSteeringDirection = previousSteeringDirection;
        }

        float smoothedBonusCurvature = 0f;
        float smoothedBonusRotation = 0f;
        float smoothedBonusTranslation = 0f;

        if (pureApfFallback)
        {
            previousBonusGains = Vector3.zero;
        }

        float targetCurvature;
        float targetRotation;
        float targetTranslation;
        if (pureApfFallback)
        {
            targetCurvature = baseControl.curvatureDegrees;
            targetRotation = baseControl.rotationDegrees;
            targetTranslation = baseControl.translationGain;
        }
        else
        {
            float bonusScale = Mathf.Max(0f, effectiveAlpha - 1f);
            float curvatureBudget = predictor.gainBudget.x;
            float rotationBudget = predictor.gainBudget.y;
            bool rotationDominant = Mathf.Abs(baseControl.rotationDegrees) > Mathf.Abs(baseControl.curvatureDegrees);

            if (rotationDominant)
            {
                float rotationExtraBudget = Mathf.Max(0f, rotationBudget - Mathf.Abs(baseControl.rotationDegrees));
                float targetBonusRotation = Mathf.Sign(baseControl.rotationDegrees == 0f ? selectedSteeringDirection : baseControl.rotationDegrees)
                    * Mathf.Min(Mathf.Abs(baseControl.rotationDegrees) * bonusScale, rotationExtraBudget);
                smoothedBonusRotation = Mathf.Lerp(previousBonusGains.y, targetBonusRotation, gainSmoothingFactor);
            }
            else
            {
                float curvatureExtraBudget = Mathf.Max(0f, curvatureBudget - Mathf.Abs(baseControl.curvatureDegrees));
                float targetBonusCurvature = Mathf.Sign(baseControl.curvatureDegrees == 0f ? selectedSteeringDirection : baseControl.curvatureDegrees)
                    * Mathf.Min(Mathf.Abs(baseControl.curvatureDegrees) * bonusScale, curvatureExtraBudget);
                smoothedBonusCurvature = Mathf.Lerp(previousBonusGains.x, targetBonusCurvature, gainSmoothingFactor);
            }

            previousBonusGains = new Vector3(smoothedBonusCurvature, smoothedBonusRotation, smoothedBonusTranslation);

            targetCurvature = baseControl.curvatureDegrees + smoothedBonusCurvature;
            targetRotation = baseControl.rotationDegrees + smoothedBonusRotation;
            targetTranslation = baseControl.translationGain + smoothedBonusTranslation;
        }

        return new Vector3(targetCurvature, targetRotation, targetTranslation);
    }

    // 应用调度的增益：当前实现对齐 ThomasAPF，旋转与曲率二选一。
    private void ApplyScheduledGains(Vector3 scheduledGains)
    {
        bool rotationDominant = Mathf.Abs(scheduledGains.y) > Mathf.Abs(scheduledGains.x);
        float appliedCurvature = rotationDominant ? 0f : scheduledGains.x;
        float appliedRotation = rotationDominant ? scheduledGains.y : 0f;
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
        lastBonusGains = previousBonusGains;
        lastFinalAppliedGains = previousAppliedGains;
        lastBoundaryDistance = currentState.nearestBoundaryDistance;
        lastPredictedBoundaryDistance = currentState.predictedBoundaryDistance;
        lastGlobalSafeStrength = ComputeGlobalSafeField(currentState).strength;
        lastDirectionalConsistency = predictor.directionalConsistency;
        lastSteerDirection = selectedSteeringDirection;
        lastUsedCriticalFallback = usedCriticalFallback;
        lastDecisionSummary = string.Format(
            "O={0:F2}, consistency={1}, boundary=({2:F2}->{3:F2}), globalSafe={4:F2}, budget=({5:F2},{6:F2},{7:F2}), base=({8:F2},{9:F2},{10:F2}), bonus=({11:F2},{12:F2},{13:F2}), final=({14:F2},{15:F2},{16:F2}), naturalTurn={17}, decel={18}, criticalFallback={19}, postResetBoost={20:F2}",
            predictor.opportunityScore,
            predictor.directionalConsistency,
            currentState.nearestBoundaryDistance,
            currentState.predictedBoundaryDistance,
            lastGlobalSafeStrength,
            predictor.gainBudget.x,
            predictor.gainBudget.y,
            predictor.gainBudget.z,
            baseControl.curvatureDegrees,
            baseControl.rotationDegrees,
            baseControl.translationGain,
            previousBonusGains.x,
            previousBonusGains.y,
            previousBonusGains.z,
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

    // 当前版本的 OAR v2 采用 APF 风格全局安全方向作为 subtle redirector 的底层骨架。
    // 机会层只调制增益强度，不再覆盖这个方向。
    private Vector2 ComputeDesiredFacingDirection(TemporalState currentState, Vector2 apfDirection)
    {
        if (apfDirection.sqrMagnitude > Utilities.eps)
        {
            return apfDirection.normalized;
        }

        Vector2 centerDirection = GetTrackingSpaceCentroid() - currentState.positionReal;
        if (centerDirection.sqrMagnitude <= Utilities.eps)
        {
            centerDirection = currentState.forwardReal;
        }
        centerDirection.Normalize();
        return centerDirection;
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

    private ThomasApfBackbone ComputeThomasApfBackbone(TemporalState currentState)
    {
        List<Vector2> nearestPosList = new List<Vector2>();
        Vector2 currPosReal = currentState.positionReal;

        for (int i = 0; i < globalConfiguration.trackingSpacePoints.Count; i++)
        {
            Vector2 p = globalConfiguration.trackingSpacePoints[i];
            Vector2 q = globalConfiguration.trackingSpacePoints[(i + 1) % globalConfiguration.trackingSpacePoints.Count];
            Vector2 nearestPos = Utilities.GetNearestPos(currPosReal, new List<Vector2> { p, q });
            nearestPosList.Add(nearestPos);
        }

        foreach (var obstacle in globalConfiguration.obstaclePolygons)
        {
            Vector2 nearestPos = Utilities.GetNearestPos(currPosReal, obstacle);
            nearestPosList.Add(nearestPos);
        }

        foreach (var user in globalConfiguration.redirectedAvatars)
        {
            MovementManager otherMovementManager = user.GetComponent<MovementManager>();
            if (otherMovementManager == null || otherMovementManager.avatarId == redirectionManager.movementManager.avatarId)
            {
                continue;
            }

            Vector2 nearestPos = Utilities.FlattenedPos2D(user.GetComponent<RedirectionManager>().currPosReal);
            nearestPosList.Add(nearestPos);
        }

        float repulsiveForce = 0f;
        Vector2 negativeGradient = Vector2.zero;
        foreach (Vector2 obstaclePosition in nearestPosList)
        {
            float distance = (currPosReal - obstaclePosition).magnitude;
            if (distance <= Utilities.eps)
            {
                continue;
            }

            repulsiveForce += 1f / distance;

            Vector2 gradientContribution =
                -Mathf.Pow(Mathf.Pow(currPosReal.x - obstaclePosition.x, 2) + Mathf.Pow(currPosReal.y - obstaclePosition.y, 2), -3f / 2f)
                * (currPosReal - obstaclePosition);
            negativeGradient += -gradientContribution;
        }

        if (negativeGradient.sqrMagnitude > Utilities.eps)
        {
            negativeGradient.Normalize();
        }

        return new ThomasApfBackbone
        {
            negativeGradient = negativeGradient,
            repulsiveForce = repulsiveForce
        };
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
                direction = Vector2.zero,
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

    private int ComputeDirectionalConsistency(Vector2 desiredFacingDirection, float averageAngularSpeed)
    {
        if (desiredFacingDirection.sqrMagnitude <= Utilities.eps || averageAngularSpeed < rotationThresholdDegreesPerSecond)
        {
            return 0;
        }

        float signedAngleToTarget = Utilities.GetSignedAngle(
            Utilities.UnFlatten(Utilities.FlattenedDir2D(redirectionManager.currDirReal)),
            Utilities.UnFlatten(desiredFacingDirection.normalized));
        if (Mathf.Abs(signedAngleToTarget) <= Utilities.eps || Mathf.Abs(redirectionManager.deltaDir) <= Utilities.eps)
        {
            return 0;
        }

        return Mathf.Sign(redirectionManager.deltaDir) == Mathf.Sign(signedAngleToTarget) ? 1 : -1;
    }

    private float GetSoftModulationBeta(PredictorOutput predictor)
    {
        if (predictor.criticalBoundaryRisk)
        {
            return 1f;
        }

        if (predictor.opportunityScore < lowOpportunityThreshold)
        {
            if (predictor.directionalConsistency < 0)
            {
                return lowOpportunityConflictBeta;
            }

            return lowOpportunityNeutralBeta;
        }

        if (predictor.opportunityScore > 0.7f)
        {
            if (predictor.directionalConsistency > 0)
            {
                return highOpportunityAlignedBeta;
            }

            return highOpportunityNeutralBeta;
        }

        return Mathf.Max(1f, neutralOpportunityBeta);
    }

    private float ComputeCombinedBoundaryRisk(TemporalState currentState)
    {
        float instantRisk = ComputeBoundaryRisk(currentState.nearestBoundaryDistance);
        float predictedRisk = ComputeBoundaryRisk(currentState.predictedBoundaryDistance);
        return Mathf.Max(instantRisk, Mathf.Lerp(instantRisk, predictedRisk, predictionWeight));
    }

    private float ComputeWaypointProximityScore(float distanceToWaypoint)
    {
        if (waypointOpportunityNearDistance <= Utilities.eps)
        {
            return 0f;
        }

        return 1f - Mathf.Clamp01(distanceToWaypoint / waypointOpportunityNearDistance);
    }

    private float ComputeUpcomingTurnScore(TemporalState currentState)
    {
        List<Vector2> waypoints = movementManager.waypoints;
        if (waypoints == null || waypoints.Count < 2)
        {
            return 0f;
        }

        int waypointIndex = movementManager.waypointIterator;
        if (waypointIndex < 0 || waypointIndex >= waypoints.Count - 1)
        {
            return 0f;
        }

        Vector2 currentTarget = waypoints[waypointIndex];
        Vector2 currentSegmentDirection = waypointIndex > 0
            ? currentTarget - waypoints[waypointIndex - 1]
            : currentTarget - currentState.positionReal;
        Vector2 nextSegmentDirection = waypoints[waypointIndex + 1] - currentTarget;

        if (currentSegmentDirection.sqrMagnitude <= Utilities.eps || nextSegmentDirection.sqrMagnitude <= Utilities.eps)
        {
            return 0f;
        }

        float turnAngle = Vector2.Angle(currentSegmentDirection, nextSegmentDirection);
        return NormalizeRange(turnAngle, waypointTurnLowDegrees, waypointTurnHighDegrees);
    }

    private float ComputeBoundaryRiskTrendScore()
    {
        if (stateHistory.Count < 2)
        {
            return 0f;
        }

        TemporalState currentState = stateHistory[stateHistory.Count - 1];
        TemporalState previousState = stateHistory[stateHistory.Count - 2];
        float currentRisk = ComputeCombinedBoundaryRisk(currentState);
        float previousRisk = ComputeCombinedBoundaryRisk(previousState);
        float riskRise = Mathf.Max(0f, currentRisk - previousRisk);
        return NormalizeRange(riskRise, boundaryRiskRiseLow, boundaryRiskRiseHigh);
    }

    private float ComputeApfDirectionChangeScore(TemporalState currentState)
    {
        if (stateHistory.Count < 2)
        {
            return 0f;
        }

        Vector2 currentDirection = ComputeThomasApfBackbone(currentState).negativeGradient;
        TemporalState previousState = stateHistory[stateHistory.Count - 2];
        Vector2 previousDirection = ComputeThomasApfBackbone(previousState).negativeGradient;

        if (currentDirection.sqrMagnitude <= Utilities.eps || previousDirection.sqrMagnitude <= Utilities.eps)
        {
            return 0f;
        }

        float directionChange = Vector2.Angle(currentDirection, previousDirection);
        return NormalizeRange(directionChange, apfDirectionChangeLowDegrees, apfDirectionChangeHighDegrees);
    }

    private float ComputeTimePressureScore(TemporalState currentState)
    {
        if (currentState.speed <= movementThresholdMetersPerSecond)
        {
            return 0f;
        }

        float timeToBoundary = currentState.predictedBoundaryDistance / Mathf.Max(currentState.speed, Utilities.eps);
        if (timeToBoundaryHighSeconds <= timeToBoundaryLowSeconds)
        {
            return timeToBoundary <= timeToBoundaryLowSeconds ? 1f : 0f;
        }

        return 1f - Mathf.Clamp01(
            Mathf.InverseLerp(timeToBoundaryLowSeconds, timeToBoundaryHighSeconds, timeToBoundary));
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
