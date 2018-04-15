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
        this.StartMarker = true;
        this.AStar = true;
        this.points = {
            'A': null,
            'B': null
        }
        var green = new THREE.Color( 0x40C843 );



        document.body.appendChild( this._renderer.domElement );

        window.addEventListener( 'resize', this.onWindowResize, false );
        window.addEventListener( 'mousemove', this.onMouseMove, false );
        window.addEventListener( 'mousedown', this.onMouseLeftClick, false );
    }

    get renderer(){
        return this._renderer;
    }

    get camera(){
        return this._camera
    }

    get scene(){
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

        this.hx_scene.camera.aspect = window.innerWidth / window.innerHeight;
        this.hx_scene._camera.updateProjectionMatrix();
    
        this.hx_scene._renderer.setSize( window.innerWidth*.75, window.innerHeight*.75 );
    
    }

    onMouseMove( event ) {
        // calculate mouse position in normalized device coordinates
        // (-1 to +1) for both components
        var rect = this.hx_scene.renderer.domElement.getBoundingClientRect();
        this.hx_scene.mouse_.x = ( ( event.clientX - rect.left ) / ( rect.width - rect.left ) ) * 2 - 1;
        this.hx_scene.mouse_.y = - ( ( event.clientY - rect.top ) / ( rect.bottom - rect.top) ) * 2 + 1;
    }

    
    onMouseLeftClick( event ) {
        // calculate objects intersecting the picking ray
        
        var intersects = this.hx_scene.raycaster.intersectObjects( this.hx_scene._scene.children );
            
            intersects[ intersects.length-1 ].object.selected = !intersects[ intersects.length-1 ].object.selected;

            console.log(intersects[ intersects.length-1 ].object.pos);

            this.hx_scene.cellNeighbor(intersects[ intersects.length-1 ].object.pos, false)
            
            if(intersects[ intersects.length-1 ].object.selected == true){
                
                if(intersects[ intersects.length-1 ].object.cell == true){ //if a cell toggle to red
                    intersects[ intersects.length-1 ].object.material.opacity = 1
                    intersects[ intersects.length-1 ].object.children = intersects[ intersects.length-1 ].object.oldChildren
                    intersects[ intersects.length-1 ].object.material.transparent = false;
                    intersects[ intersects.length-1 ].object.material.color.set( 0xFF0000 );
                }else{ //if a tile turn transparent
                    if(this.AStar = true){
                        this.StartMarker = !this.StartMarker;
                        if(!this.StartMarker){
                            intersects[ intersects.length-1 ].object.material.color.set( 0x0047FF );
                            var x,y;
                            x = intersects[ intersects.length-1 ].object.pos.x;
                            y = intersects[ intersects.length-1 ].object.pos.y;
                            this.hx_scene.points['B'] = [x,y];
                            hx_grid.calculateDistance(this.hx_scene.points['A'], this.hx_scene.points['B']);
                        }else{
                            intersects[ intersects.length-1 ].object.material.color.set( 0xFF0000 );  
                            var x,y;
                            x = intersects[ intersects.length-1 ].object.pos.x;
                            y = intersects[ intersects.length-1 ].object.pos.y;
                            this.hx_scene.points['A'] = [x,y];
                        }
                    }
                    //intersects[ intersects.length-1 ].object.material.opacity = 0
                    //intersects[ intersects.length-1 ].object.material.transparent = true;
                }

            }else{

                if(intersects[ intersects.length-1 ].object.cell == true){ //if a cell turn back to transparent
                    intersects[ intersects.length-1 ].object.material.opacity = 0
                    intersects[ intersects.length-1 ].object.material.transparent = true;
                    intersects[ intersects.length-1 ].object.material.color.set( 0xFFFFFF ); 
                }else{ //if a tile turn back green
                    intersects[ intersects.length-1 ].object.material.opacity = 1
                    intersects[ intersects.length-1 ].object.material.transparent = false;
                    intersects[ intersects.length-1 ].object.material.color.set( 0x40C843 ); 
                }           
            }
    }

    cellNeighbor(pos, mark){
        //console.log(hx_grid.findNeighbor(pos, mark, "object"));
    }


}