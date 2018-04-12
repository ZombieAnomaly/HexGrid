class hx_Scene {
    
    constructor(){

        this._scene = new THREE.Scene();
        this._camera = new THREE.PerspectiveCamera( 75, window.innerWidth/window.innerHeight, 0.1, 1000 );
        this._renderer = new THREE.WebGLRenderer({ alpha: true, antialias: true });
        this._renderer.setSize( window.innerWidth*.75, window.innerHeight*.75);

        this.controls = new THREE.OrbitControls( this._camera );
        this.controls.update();
        this.raycaster = new THREE.Raycaster();
        this.mouse_ = new THREE.Vector2();
        this.mouse_.x = 0
        this.mouse_.y = 0
        var green = new THREE.Color( 0x40C843 );



        document.body.appendChild( this._renderer.domElement );

        window.addEventListener( 'resize', this.onWindowResize, false );
        window.addEventListener( 'mousemove', this.onMouseMove, false );
        window.addEventListener( 'mousedown', this.onMouseLeftClick, false );
    }

    renderer(){
        return this._renderer;
    }

    camera(){
        return this._camera
    }

    scene(){
        return this._scene
    }

    add(obj){
        this._scene.add(obj);
    }

    controlsUpdate(){

        this.controls.update();
        // update the picking ray with the camera and mouse position
        this.raycaster.setFromCamera( this.mouse_, this._camera );


    }

    onWindowResize(){

        this.hx_scene.camera().aspect = window.innerWidth / window.innerHeight;
        this.hx_scene._camera.updateProjectionMatrix();
    
        this.hx_scene._renderer.setSize( window.innerWidth*.75, window.innerHeight*.75 );
    
    }

    onMouseMove( event ) {
        // calculate mouse position in normalized device coordinates
        // (-1 to +1) for both components
        var rect = this.hx_scene.renderer().domElement.getBoundingClientRect();
        this.hx_scene.mouse_.x = ( ( event.clientX - rect.left ) / ( rect.width - rect.left ) ) * 2 - 1;
        this.hx_scene.mouse_.y = - ( ( event.clientY - rect.top ) / ( rect.bottom - rect.top) ) * 2 + 1;
    }

    onMouseLeftClick( event ) {
        // calculate objects intersecting the picking ray
        var intersects = this.hx_scene.raycaster.intersectObjects( this.hx_scene._scene.children );
        for ( var i = 0; i < intersects.length; i++ ) {

            if(intersects[ i ].object.selected == false){
                intersects[ i ].object.material.opacity = 1
                intersects[ i ].object.material.transparent = false;
                intersects[ i ].object.material.color.set( 0xff0000 );
                intersects[ i ].object.selected = true
            }else{
                intersects[ i ].object.material.opacity = 0
                intersects[ i ].object.material.transparent = true;
                intersects[ i ].object.material.color.set( 0xFFFFFF );  
                intersects[ i ].object.selected = false             
            }

        }
    }
}