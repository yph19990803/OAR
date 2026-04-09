using UnityEngine;

public class OpportunityAwareResetter : Resetter
{
    [Min(0f)]
    public float minResetRotationDegrees = 30f;

    [Range(30f, 180f)]
    public float maxResetRotationDegrees = 180f;

    [Min(0.1f)]
    public float resetSteeringRatio = 1f;

    float requiredRotateSteerAngle = 0f;
    float requiredRotateAngle = 0f;
    float rotateDir = 1f;

    public override bool IsResetRequired()
    {
        return IfCollisionHappens();
    }

    public override void InitializeReset()
    {
        Vector2 targetDirection = GetTargetDirection();
        Vector2 currentDirection = Utilities.FlattenedDir2D(redirectionManager.currDirReal);
        if (targetDirection.sqrMagnitude <= Utilities.eps)
        {
            targetDirection = currentDirection;
        }

        float signedAngle = Utilities.GetSignedAngle(
            redirectionManager.currDirReal,
            Utilities.UnFlatten(targetDirection.normalized));

        rotateDir = -(int)Mathf.Sign(signedAngle);
        if (Mathf.Abs(rotateDir) < Mathf.Epsilon)
        {
            rotateDir = 1f;
        }

        float requestedRotation = Mathf.Abs(signedAngle);
        if (requestedRotation > Utilities.eps)
        {
            requestedRotation = Mathf.Clamp(requestedRotation, minResetRotationDegrees, maxResetRotationDegrees);
        }
        else
        {
            requestedRotation = 180f;
        }

        requiredRotateSteerAngle = requestedRotation;
        requiredRotateAngle = requestedRotation;

        SetHUD((int)rotateDir);
    }

    public override void InjectResetting()
    {
        float steerRotation = resetSteeringRatio * redirectionManager.deltaDir;
        if (Mathf.Abs(steerRotation) > Utilities.eps && Mathf.Sign(steerRotation) != rotateDir)
        {
            return;
        }

        if (Mathf.Abs(requiredRotateSteerAngle) <= Mathf.Abs(steerRotation) || requiredRotateAngle == 0)
        {
            InjectRotation(rotateDir * requiredRotateSteerAngle);
            requiredRotateSteerAngle = 0f;
            redirectionManager.OnResetEnd();
        }
        else
        {
            InjectRotation(steerRotation);
            requiredRotateSteerAngle -= Mathf.Abs(steerRotation);
        }
    }

    public override void EndReset()
    {
        DestroyHUD();
    }

    public override void SimulatedWalkerUpdate()
    {
        float rotateAngle = redirectionManager.GetDeltaTime() * redirectionManager.globalConfiguration.rotationSpeed;
        if (rotateAngle >= requiredRotateAngle)
        {
            rotateAngle = requiredRotateAngle;
            requiredRotateAngle = 0f;
        }
        else
        {
            requiredRotateAngle -= rotateAngle;
        }

        redirectionManager.simulatedWalker.RotateInPlace(rotateAngle * rotateDir);
    }

    private Vector2 GetTargetDirection()
    {
        var opportunityAware = redirectionManager.redirector as OpportunityAwareRDWRedirector;
        if (opportunityAware != null)
        {
            return opportunityAware.GetRecommendedResetDirection();
        }

        Debug.LogError("OpportunityAwareResetter requires OpportunityAwareRDWRedirector");
        return Utilities.FlattenedDir2D(redirectionManager.currDirReal);
    }
}
