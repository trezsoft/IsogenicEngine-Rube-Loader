var RubeLoaderComponent = IgeClass.extend({
    classId: 'RubeLoaderComponent',
    componentId: 'rubeloader',

    /**
     * @constructor
     * @param entity
     * @param options
     */
    init: function (entity, options) {
        this._entity = entity;
        this._options = options;
    },

    /**
     * Loads a .js Rube json-format file and converts to IGE,
     * creating box2d bodies,fixtures and joints.
     * @param sceneUrl
     * @param sceneName
     * @param mountScene
     * @param isIsometric
     * @param loadImages
     */
    loadRubeScene: function (sceneName, sceneUrl, mountScene, isIsometric, loadImages, callback) {
        var self = this,
            scriptElem;

        if (typeof(sceneUrl) === 'string') {
            if (!ige.isServer) {
                scriptElem = document.createElement('script');
                scriptElem.src = sceneUrl;
                scriptElem.onload = function () {
                    self.log('Rube data loaded, processing...');
                    callback(self.loadSceneIntoWorld(window[sceneName], mountScene, isIsometric, loadImages));
                };
                document.getElementsByTagName('head')[0].appendChild(scriptElem);
            } else {
                this.log('URL-based Rube data is only available client-side. If you want to load Rube data on the server please include the Rube Scene file in your ServerConfig.js file and then specify the scene data object instead of the URL.', 'error');
            }
        } else {
            callback(self.loadSceneIntoWorld(window[sceneName], mountScene, isIsometric, loadImages));
        }
    },

    loadSceneIntoWorld: function (worldJso, mountScene, isIsometric, loadImages) {
        var success = true, i;

        var loadedBodies = [];
        if (worldJso.hasOwnProperty('body')) {
            for (i = 0; i < worldJso.body.length; i++) {
                var bodyJso = worldJso.body[i];
                var body = this.loadBodyFromRUBE(bodyJso, mountScene, isIsometric);
                if (body)
                    loadedBodies.push(body);
                else
                    success = false;
            }
        }

        var loadedJoints = [];
        if (worldJso.hasOwnProperty('joint')) {
            for (i = 0; i < worldJso.joint.length; i++) {
                var jointJso = worldJso.joint[i];
                var joint = this.loadJointFromRUBE(jointJso, mountScene, loadedBodies);
                if (joint)
                    loadedJoints.push(joint);
            }
        }

        if (loadImages) {
            var loadedImages = [];
            if (worldJso.hasOwnProperty('image')) {
                for (i = 0; i < worldJso.image.length; i++) {
                    var imageJso = worldJso.image[i];
                    var image = this.loadImageFromRUBE(imageJso, mountScene, loadedBodies, isIsometric);
                    if (image)
                        loadedImages.push(image);
                    else
                        success = false;
                }

            }
        }

        return success;

    },

    loadBodyFromRUBE: function (bodyJso, mountScene, isIsometric) {

        if (!bodyJso.hasOwnProperty('type')) {
            console.log("Body does not have a 'type' property");
            return null;
        }

        var bd = new Box2D.Dynamics.b2BodyDef();
        if (bodyJso.type == 2)
            bd.type = 'dynamic';
        else if (bodyJso.type == 1)
            bd.type = 'kinematic';
        if (bodyJso.hasOwnProperty('angle'))
            bd.angle = bodyJso.angle * -1;
        if (bodyJso.hasOwnProperty('angularVelocity'))
            bd.angularVelocity = bodyJso.angularVelocity * -1;
        if (bodyJso.hasOwnProperty('active'))
            bd.awake = bodyJso.active;
        if (bodyJso.hasOwnProperty('fixedRotation'))
            bd.fixedRotation = bodyJso.fixedRotation;
        if (bodyJso.hasOwnProperty('linearVelocity') && bodyJso.linearVelocity instanceof Object)
            bd.linearVelocity.SetV(new ige.box2d.b2Vec2(bodyJso.linearVelocity.x, bodyJso.linearVelocity.y * -1));
        if (bodyJso.hasOwnProperty('awake'))
            bd.awake = bodyJso.awake;
        else
            bd.awake = false;

        var physicsEntity = new IgeEntityBox2d();
        physicsEntity.box2dBody(bd);
        physicsEntity.isometric(isIsometric);

        if (bodyJso.hasOwnProperty('name')) {
            physicsEntity._box2dBody.name = bodyJso.name;
            physicsEntity.id(bodyJso.name);
        }

        physicsEntity.mount(ige.$(mountScene));

        if (bodyJso.hasOwnProperty('position') && bodyJso.position instanceof Object)
            physicsEntity.translateTo(bodyJso.position.x * ige.box2d._scaleRatio, bodyJso.position.y * ige.box2d._scaleRatio * -1, 0);

        if (bodyJso.hasOwnProperty('fixture')) {
            for (var k = 0; k < bodyJso['fixture'].length; k++) {
                var fixtureJso = bodyJso['fixture'][k];
                this.loadFixtureFromRUBE(physicsEntity._box2dBody, fixtureJso);
            }
        }

        if (bodyJso.hasOwnProperty('customProperties'))
            physicsEntity._box2dBody.customProperties = bodyJso.customProperties;

        return physicsEntity._box2dBody;
    },
    loadFixtureFromRUBE: function (body, fixtureJso) {

        var fixture = null;
        var fd = new Box2D.Dynamics.b2FixtureDef();
        if (fixtureJso.hasOwnProperty('friction'))
            fd.friction = fixtureJso.friction;
        if (fixtureJso.hasOwnProperty('density'))
            fd.density = fixtureJso.density;
        if (fixtureJso.hasOwnProperty('restitution'))
            fd.restitution = fixtureJso.restitution;
        if (fixtureJso.hasOwnProperty('sensor'))
            fd.isSensor = fixtureJso.sensor;
        if (fixtureJso.hasOwnProperty('filter-categoryBits'))
            fd.filter.categoryBits = fixtureJso['filter-categoryBits'];
        if (fixtureJso.hasOwnProperty('filter-maskBits'))
            fd.filter.maskBits = fixtureJso['filter-maskBits'];
        if (fixtureJso.hasOwnProperty('filter-groupIndex'))
            fd.filter.groupIndex = fixtureJso['filter-groupIndex'];
        if (fixtureJso.hasOwnProperty('circle')) {
            fd.shape = new Box2D.Collision.Shapes.b2CircleShape();
            fd.shape.m_radius = fixtureJso.circle.radius;
            if (fixtureJso.circle.center)
                fd.shape.m_p.SetV(new ige.box2d.b2Vec2(fixtureJso.circle.center.x, fixtureJso.circle.center.y * -1));
            fixture = body.CreateFixture(fd);
            if (fixtureJso.name)
                fixture.name = fixtureJso.name;
        }
        else if (fixtureJso.hasOwnProperty('polygon')) {
            fd.shape = new Box2D.Collision.Shapes.b2PolygonShape();
            var verts = [];

            for (var v = fixtureJso.polygon.vertices.x.length - 1; v > -1; v--)
                verts.push(new ige.box2d.b2Vec2(fixtureJso.polygon.vertices.x[v], fixtureJso.polygon.vertices.y[v] * -1));
            fd.shape.SetAsArray(verts, verts.length);
            fixture = body.CreateFixture(fd);
            if (fixture && fixtureJso.name)
                fixture.name = fixtureJso.name;
        }
        else if (fixtureJso.hasOwnProperty('chain')) {
            fd.shape = new Box2D.Collision.Shapes.b2PolygonShape();
            var lastVertex = new Box2D.Common.Math.b2Vec2(0, 0);
            for (var v = 0; v < fixtureJso.chain.vertices.x.length; v++) {
                var thisVertex = new Box2D.Common.Math.b2Vec2(fixtureJso.chain.vertices.x[v], fixtureJso.chain.vertices.y[v] * -1);
                if (v > 0) {
                    fd.shape.SetAsEdge(lastVertex, thisVertex);
                    fixture = body.CreateFixture(fd);
                    if (fixtureJso.name)
                        fixture.name = fixtureJso.name;
                }
                lastVertex = thisVertex;
            }
        }
        else {
            console.log("Could not find shape type for fixture");
        }

        if (fixture) {
            if (fixtureJso.hasOwnProperty('customProperties'))
                fixture.customProperties = fixtureJso.customProperties;
        }
    },

    getVectorValue: function (val) {
        if (val instanceof Object)
            return new Box2D.Common.Math.b2Vec2(val.x, val.y * -1);
        else
            return { x: 0, y: 0 };
    },

    loadJointCommonProperties: function (jd, jointJso, loadedBodies) {
        jd.bodyA = loadedBodies[jointJso.bodyA];
        jd.bodyB = loadedBodies[jointJso.bodyB];
        jd.localAnchorA.SetV(this.getVectorValue(jointJso.anchorA));
        jd.localAnchorB.SetV(this.getVectorValue(jointJso.anchorB));
        if (jointJso.collideConnected)
            jd.collideConnected = jointJso.collideConnected;
    },

    loadJointFromRUBE: function (jointJso, mountScene, loadedBodies) {
        if (!jointJso.hasOwnProperty('type')) {
            console.log("Joint does not have a 'type' property");
            return null;
        }
        if (jointJso.bodyA >= loadedBodies.length) {
            console.log("Index for bodyA is invalid: " + jointJso.bodyA);
            return null;
        }
        if (jointJso.bodyB >= loadedBodies.length) {
            console.log("Index for bodyB is invalid: " + jointJso.bodyB);
            return null;
        }

        var joint = null, jd = null;
        if (jointJso.type == "revolute") {
            jd = new Box2D.Dynamics.Joints.b2RevoluteJointDef();
            this.loadJointCommonProperties(jd, jointJso, loadedBodies);
            if (jointJso.hasOwnProperty('refAngle'))
                jd.referenceAngle = jointJso.refAngle * -1;
            if (jointJso.hasOwnProperty('lowerLimit'))
                jd.lowerAngle = jointJso.lowerLimit;
            if (jointJso.hasOwnProperty('upperLimit'))
                jd.upperAngle = jointJso.upperLimit;
            if (jointJso.hasOwnProperty('maxMotorTorque'))
                jd.maxMotorTorque = jointJso.maxMotorTorque * ige.box2d._scaleRatio;
            if (jointJso.hasOwnProperty('motorSpeed'))
                jd.motorSpeed = jointJso.motorSpeed * -1;
            if (jointJso.hasOwnProperty('enableLimit'))
                jd.enableLimit = jointJso.enableLimit;
            if (jointJso.hasOwnProperty('enableMotor'))
                jd.enableMotor = jointJso.enableMotor;
            joint = ige.box2d._world.CreateJoint(jd);
        }
        else if (jointJso.type == "distance" || jointJso.type == "rope") {
            if (jointJso.type == "rope")
                console.log("Replacing unsupported rope joint with distance joint!");
            jd = new Box2D.Dynamics.Joints.b2DistanceJointDef();
            this.loadJointCommonProperties(jd, jointJso, loadedBodies);
            if (jointJso.hasOwnProperty('length'))
                jd.length = jointJso.length;
            if (jointJso.hasOwnProperty('dampingRatio'))
                jd.dampingRatio = jointJso.dampingRatio;
            if (jointJso.hasOwnProperty('frequency'))
                jd.frequencyHz = jointJso.frequency;
            joint = ige.box2d._world.CreateJoint(jd);
        }
        else if (jointJso.type == "prismatic") {
            jd = new Box2D.Dynamics.Joints.b2PrismaticJointDef();
            this.loadJointCommonProperties(jd, jointJso, loadedBodies);
            if (jointJso.hasOwnProperty('localAxisA'))
                jd.localAxisA.SetV(this.getVectorValue(jointJso.localAxisA));
            if (jointJso.hasOwnProperty('refAngle'))
                jd.referenceAngle = jointJso.refAngle * -1;
            if (jointJso.hasOwnProperty('enableLimit'))
                jd.enableLimit = jointJso.enableLimit;
            if (jointJso.hasOwnProperty('lowerLimit'))
                jd.lowerTranslation = jointJso.lowerLimit;
            if (jointJso.hasOwnProperty('upperLimit'))
                jd.upperTranslation = jointJso.upperLimit;
            if (jointJso.hasOwnProperty('enableMotor'))
                jd.enableMotor = jointJso.enableMotor;
            if (jointJso.hasOwnProperty('maxMotorForce'))
                jd.maxMotorForce = jointJso.maxMotorForce * ige.box2d._scaleRatio;
            if (jointJso.hasOwnProperty('motorSpeed'))
                jd.motorSpeed = jointJso.motorSpeed;
            joint = ige.box2d._world.CreateJoint(jd);
        }
        else if (jointJso.type == "wheel") {
            //Make a fake wheel joint using a line joint and a distance joint.
            //Return the line joint because it has the linear motor controls.
            //Use ApplyTorque on the bodies to spin the wheel...

            jd = new Box2D.Dynamics.Joints.b2DistanceJointDef();
            this.loadJointCommonProperties(jd, jointJso, loadedBodies);
            jd.length = 0.0;
            if (jointJso.hasOwnProperty('springDampingRatio'))
                jd.dampingRatio = jointJso.springDampingRatio;
            if (jointJso.hasOwnProperty('springFrequency'))
                jd.frequencyHz = jointJso.springFrequency;
            ige.box2d._world.CreateJoint(jd);

            jd = new Box2D.Dynamics.Joints.b2LineJointDef();
            this.loadJointCommonProperties(jd, jointJso, loadedBodies);
            if (jointJso.hasOwnProperty('localAxisA'))
                jd.localAxisA.SetV(this.getVectorValue(jointJso.localAxisA));

            joint = ige.box2d._world.CreateJoint(jd);
        }
        else if (jointJso.type == "friction") {
            jd = new Box2D.Dynamics.Joints.b2FrictionJointDef();
            this.loadJointCommonProperties(jd, jointJso, loadedBodies);
            if (jointJso.hasOwnProperty('maxForce'))
                jd.maxForce = jointJso.maxForce * 5;
            if (jointJso.hasOwnProperty('maxTorque'))
                jd.maxTorque = jointJso.maxTorque * 3.5;
            joint = ige.box2d._world.CreateJoint(jd);
        }
        else if (jointJso.type == "weld") {
            jd = new Box2D.Dynamics.Joints.b2WeldJointDef();
            this.loadJointCommonProperties(jd, jointJso, loadedBodies);
            if (jointJso.hasOwnProperty('refAngle'))
                jd.referenceAngle = jointJso.refAngle * -1;
            if (jointJso.hasOwnProperty('dampingRatio'))
                jd.dampingRatio = jointJso.dampingRatio;
            if (jointJso.hasOwnProperty('frequency'))
                jd.frequencyHz = jointJso.frequency;
            joint = ige.box2d._world.CreateJoint(jd);
        }
        else {
            console.log("Unsupported joint type: " + jointJso.type);
            console.log(jointJso);
        }
        if (joint) {
            if (jointJso.name)
                joint.name = jointJso.name;
            if (jointJso.hasOwnProperty('customProperties'))
                joint.customProperties = jointJso.customProperties;
        }
        return joint;
    },
    loadImageFromRUBE: function (imageJso, mountScene, loadedBodies, isIsometric) {
        var filename = "";

        if (imageJso.hasOwnProperty('body') && imageJso.body >= 0) {
            filename = this.GetFilename(imageJso.file, false);
            var imageEntity = new IgeEntity()
                .id(imageJso.name + '-' + ige.newIdHex())
                .texture(ige.client.gameTextures[filename]);

            imageEntity.mount(ige.$(loadedBodies[imageJso.body].name));
            imageEntity.heightByTile(1, true);

            if (imageJso.hasOwnProperty('scale')) {
                imageEntity.scaleBy(imageJso.scale * ige.box2d._scaleRatio, imageJso.scale * ige.box2d._scaleRatio, 0);
            }
            if (imageJso.hasOwnProperty('aspectScale')) {
                if (imageJso.aspectScale != 1)
                    imageEntity.scaleBy(imageJso.scale * imageJso.aspectScale, 0, 0);
            }
            if (imageJso.hasOwnProperty('angle')) {
                imageEntity.rotateTo(0, 0, imageJso.angle * -1);
            }
            if (imageJso.center.hasOwnProperty('x')) {
                imageEntity.translateTo(imageJso.center.x * ige.box2d._scaleRatio, imageJso.center.y * ige.box2d._scaleRatio * -1, 0)
            }

            imageEntity.isometric(isIsometric)
        }
        else {
            filename = this.GetFilename(imageJso.file, false);
            var backgroundImage = new IgeEntity()
                .id(imageJso.name + '-' + ige.newIdHex())
                .texture(ige.client.gameTextures[filename]);

            if (imageJso.center.hasOwnProperty('x')) {
                backgroundImage.translateTo(imageJso.center.x * ige.box2d._scaleRatio, imageJso.center.y * ige.box2d._scaleRatio * -1, 0)
            }

            backgroundImage.mount(ige.$(mountScene));
            backgroundImage.heightByTile(.72, true);

            if (imageJso.hasOwnProperty('scale')) {
                if (imageJso.hasOwnProperty('aspectScale') && imageJso.aspectScale != 1) {
                    backgroundImage.scaleBy(imageJso.scale * imageJso.aspectScale, imageJso.scale, 0);
                }
                else {
                    backgroundImage.scaleBy(imageJso.scale, imageJso.scale, 0);
                }
            }
            if (imageJso.hasOwnProperty('angle')) {
                backgroundImage.rotateTo(0, 0, imageJso.angle * -1);
            }

            backgroundImage.isometric(isIsometric);
        }

        return imageJso;
    },
    GetFilename: function (url, includeExtension) {
        if (url) {
            var m;
            if (includeExtension) {
                m = url.replace(/^.*[\\\/]/, '');
            }
            else {
                m = url.toString().match(/.*\/(.+?)\./);
            }

            if (m && m.length > 1) {
                return m[1];
            }
        }
        return '';
    }
});

Object.prototype.hasOwnProperty = function (property) {
    return typeof(this[property]) !== 'undefined'
};

if (typeof(module) !== 'undefined' && typeof(module.exports) !== 'undefined') {
    module.exports = RubeLoaderComponent;
}