#include "CollisionObject.h"

using namespace physic;


bool IntersectionTests::sphereAndSphere(
    const CollisionSphere &one,
    const CollisionSphere &two)
{
    // Find the vector between the objects
    Vector3 midline = one.getAxis(3) - two.getAxis(3);

    // See if it is large enough.
    return midline.SquareMagnitude() <
        (one.radius+two.radius)*(one.radius+two.radius);
}

static inline float transformToAxis(
    const CollisionBox &box,
    const Vector3 &axis
    )
{
    return
        box.halfSize.x * abs(axis * box.getAxis(0)) +
        box.halfSize.y * abs(axis * box.getAxis(1)) +
        box.halfSize.z * abs(axis * box.getAxis(2));
}

/**
 * This function checks if the two boxes overlap
 * along the given axis. The final parameter toCentre
 * is used to pass in the vector between the boxes centre
 * points, to avoid having to recalculate it each time.
 */
static inline bool overlapOnAxis(
    const CollisionBox &one,
    const CollisionBox &two,
    const Vector3 &axis,
    const Vector3 &toCentre
    )
{
    // Project the half-size of one onto axis
    float oneProject = transformToAxis(one, axis);
    float twoProject = transformToAxis(two, axis);

    // Project this onto the axis
    float distance = abs(toCentre * axis);

    // Check for overlap
    return (distance < oneProject + twoProject);
}


#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), toCentre)

bool IntersectionTests::boxAndBox(const CollisionBox &one,const CollisionBox &two)
{
    // Find the vector between the two centres
    Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

    return (
        // Check on box one's axes first
        TEST_OVERLAP(one.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(2)) &&

        // And on two's
        TEST_OVERLAP(two.getAxis(0)) &&
        TEST_OVERLAP(two.getAxis(1)) &&
        TEST_OVERLAP(two.getAxis(2)) &&

        // Now on the cross products
        TEST_OVERLAP(one.getAxis(0) % two.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(0) % two.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(0) % two.getAxis(2)) &&
        TEST_OVERLAP(one.getAxis(1) % two.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(1) % two.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(1) % two.getAxis(2)) &&
        TEST_OVERLAP(one.getAxis(2) % two.getAxis(0)) &&
        TEST_OVERLAP(one.getAxis(2) % two.getAxis(1)) &&
        TEST_OVERLAP(one.getAxis(2) % two.getAxis(2))
    );
}
#undef TEST_OVERLAP

bool IntersectionTests::boxAndHalfSpace(const CollisionBox &box, const CollisionPlane &plane )
{
    // Work out the projected radius of the box onto the plane direction
    float projectedRadius = transformToAxis(box, plane.direction);

    // Work out how far the box is from the origin
    float boxDistance = plane.direction * box.getAxis(3) - projectedRadius;

    // Check for the intersection
    return boxDistance <= plane.offset;
}

unsigned CollisionDetector::sphereAndTruePlane(const CollisionSphere &sphere,const CollisionPlane &plane,CollisionData *data)
{
    // Make sure we have contacts
    if (data->contactsLeft <= 0) return 0;

    // Cache the sphere position
    Vector3 position = sphere.getAxis(3);

    // Find the distance from the plane
    float centreDistance = plane.direction * position - plane.offset;

    // Check if we're within radius
    if (centreDistance*centreDistance > sphere.radius*sphere.radius)
    {
        return 0;
    }

    // Check which side of the plane we're on
    Vector3 normal = plane.direction;
    float penetration = -centreDistance;
    if (centreDistance < 0)
    {
        normal *= -1;
        penetration = -penetration;
    }
    penetration += sphere.radius;

    // Create the contact - it has a normal in the plane direction.
    Contact* contact = data->contacts;
    contact->contactNormal = normal;
    contact->penetration = penetration;
    contact->contactPoint = position - plane.direction * centreDistance;
    contact->setBodyData(sphere.body,nullptr,data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::sphereAndSphere(const CollisionSphere &one,const CollisionSphere &two,CollisionData *data)
{
    // Make sure we have contacts
    if (data->contactsLeft <= 0) return 0;

    // Cache the sphere positions
    Vector3 positionOne = one.getAxis(3);
    Vector3 positionTwo = two.getAxis(3);

    // Find the vector between the objects
    Vector3 midline = positionOne - positionTwo;
    float size = midline.Magnitude();

    // See if it is large enough.
    if (size <= 0.0f || size >= one.radius+two.radius)
    {
        return 0;
    }

    // We manually create the normal, because we have the
    // size to hand.
    Vector3 normal = midline * (((float)1.0)/size);

    Contact* contact = data->contacts;
    contact->contactNormal = normal;
    contact->contactPoint = positionOne + midline * (float)0.5;
    contact->penetration = (one.radius+two.radius - size);
    contact->setBodyData(one.body, two.body,
        data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}


static inline Vector3 contactPoint(
    const Vector3 &pOne,
    const Vector3 &dOne,
    float oneSize,
    const Vector3 &pTwo,
    const Vector3 &dTwo,
    float twoSize,

    // If this is true, and the contact point is outside
    // the edge (in the case of an edge-face contact) then
    // we use one's midpoint, otherwise we use two's.
    bool useOne)
{
    Vector3 toSt, cOne, cTwo;
    float dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
    float denom, mua, mub;

    smOne = dOne.SquareMagnitude();
    smTwo = dTwo.SquareMagnitude();
    dpOneTwo = dTwo * dOne;

    toSt = pOne - pTwo;
    dpStaOne = dOne * toSt;
    dpStaTwo = dTwo * toSt;

    denom = smOne * smTwo - dpOneTwo * dpOneTwo;

    // Zero denominator indicates parrallel lines
    if (abs(denom) < 0.0001f) {
        return useOne?pOne:pTwo;
    }

    mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
    mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

    // If either of the edges has the nearest point out
    // of bounds, then the edges aren't crossed, we have
    // an edge-face contact. Our point is on the edge, which
    // we know from the useOne parameter.
    if (mua > oneSize ||
        mua < -oneSize ||
        mub > twoSize ||
        mub < -twoSize)
    {
        return useOne?pOne:pTwo;
    }
    else
    {
        cOne = pOne + dOne * mua;
        cTwo = pTwo + dTwo * mub;

        return cOne * 0.5 + cTwo * 0.5;
    }
}


unsigned CollisionDetector::boxAndSphere(
    const CollisionBox &box,
    const CollisionSphere &sphere,
    CollisionData *data
    )
{
    // Transform the centre of the sphere into box coordinates
    Vector3 centre = sphere.getAxis(3);
    Vector3 relCentre = box.transform.TransformInverse(centre);

    // Early out check to see if we can exclude the contact
    if (abs(relCentre.x) - sphere.radius > box.halfSize.x ||
        abs(relCentre.y) - sphere.radius > box.halfSize.y ||
        abs(relCentre.z) - sphere.radius > box.halfSize.z)
    {
        return 0;
    }

    Vector3 closestPt(0,0,0);
    float dist;

    // Clamp each coordinate to the box.
    dist = relCentre.x;
    if (dist > box.halfSize.x) dist = box.halfSize.x;
    if (dist < -box.halfSize.x) dist = -box.halfSize.x;
    closestPt.x = dist;

    dist = relCentre.y;
    if (dist > box.halfSize.y) dist = box.halfSize.y;
    if (dist < -box.halfSize.y) dist = -box.halfSize.y;
    closestPt.y = dist;

    dist = relCentre.z;
    if (dist > box.halfSize.z) dist = box.halfSize.z;
    if (dist < -box.halfSize.z) dist = -box.halfSize.z;
    closestPt.z = dist;

    // Check we're in contact
    dist = (closestPt - relCentre).SquareMagnitude();
    if (dist > sphere.radius * sphere.radius) return 0;

    // Compile the contact
    Vector3 closestPtWorld = box.transform.Transform(closestPt);

    Contact* contact = data->contacts;
    contact->contactNormal = (closestPtWorld - centre);
    contact->contactNormal.Normalise();
    contact->contactPoint = closestPtWorld;
    contact->penetration = sphere.radius - sqrt(dist);
    contact->setBodyData(box.body, sphere.body,
        data->friction, data->restitution);

    data->addContacts(1);
    return 1;
}

unsigned CollisionDetector::boxAndHalfSpace(
    const CollisionBox &box,
    const CollisionPlane &plane,
    CollisionData *data
    )
{
    // Make sure we have contacts
    if (data->contactsLeft <= 0) return 0;

    // Check for intersection
    if (!IntersectionTests::boxAndHalfSpace(box, plane))
    {
        return 0;
    }

    // We have an intersection, so find the intersection points.

    // Go through each combination of + and - for each half-size
    static float mults[8][3] = {{1,1,1},{-1,1,1},{1,-1,1},{-1,-1,1},
                               {1,1,-1},{-1,1,-1},{1,-1,-1},{-1,-1,-1}};

    Contact* contact = data->contacts;
    unsigned contactsUsed = 0;
    for (unsigned i = 0; i < 8; i++) {

        // Calculate the position of each vertex
        Vector3 vertexPos(mults[i][0], mults[i][1], mults[i][2]);
        vertexPos.ComponentProductUpdate(box.halfSize);
        vertexPos = box.transform.Transform(vertexPos);

        // Calculate the distance from the plane
        float vertexDistance = vertexPos * plane.direction;

        // Compare this to the plane's distance
        if (vertexDistance <= plane.offset)
        {
            // Create the contact data.

            // The contact point is halfway between the vertex and the
            // plane - we multiply the direction by half the separation
            // distance and add the vertex location.
            contact->contactPoint = plane.direction;
            contact->contactPoint *= (vertexDistance-plane.offset);
            contact->contactPoint = vertexPos;
            contact->contactNormal = plane.direction;
            contact->penetration = plane.offset - vertexDistance;

            // Write the appropriate data
            contact->setBodyData(box.body, nullptr,
                data->friction, data->restitution);

            // Move onto the next contact
            contact++;
            contactsUsed++;
            if (contactsUsed == data->contactsLeft) return contactsUsed;
        }
    }

    data->addContacts(contactsUsed);
    return contactsUsed;
}