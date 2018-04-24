class hx_Grid {
    constructor(MaxWidth,MaxLength,HeightMap){

        this.cells = {}
        this.MaxLength = MaxLength;
        this.MaxWidth = MaxWidth;
        this.HeightMap = HeightMap;
    }

    generate(){
        for(var i = 0; i < this.MaxLength; i++){
            for(var j =0; j < this.MaxWidth; j++){
                var hxCell = new hx_Cell().cell;
                var Cell = hxCell.Cell;
                Cell.pos = {
                    x: j,
                    y: i,
                    h: 0.5
                }
                Cell.selected = false;
                Cell.cell = true;
                this.add(hxCell)
                var offset = 0;
                if(Cell.pos.y % 2 != 0){
                    offset = .9
                }

                Cell.position.set((Cell.pos.x*1.75) + offset,0,Cell.pos.y*1.5)
                
                hx_scene.add(Cell)
            }
        }
        this.findNeighbors();
    }

    cleanup(){
        for(var i = 0; i < this.MaxLength; i++){
            for(var j =0; j < this.MaxWidth; j++){
                var key = String(i) + "," + String(j)
                this.cells[key].hx_cell.Cell.oldChildren = this.cells[key].hx_cell.Cell.children;
                this.cells[key].hx_cell.rmWireframe();
            }
        }
    }

    add(cell){
        var key, pos;
        pos = cell.Cell.pos;
        key = String(pos.x) + "," + String(pos.y)
        this.cells[key] = {
            hx_cell: cell,
            hx_tile: null,
            _id: this.guid(),
            outline: cell.Cell.children
           
        };
    }

    get grid(){
        return this.cells;
    }

    guid() {
        function s4() {
          return Math.floor((1 + Math.random()) * 0x10000)
            .toString(16)
            .substring(1);
        }
        return 'UID-' + s4()+s4();
      }

    removeTile(cell){
        this.cells[cell.pos].tile = null;
    }

    updateCellObject(key,hxTile){
        this.cells[key].hx_tile = hxTile
    }

    findNeighbors(){
        for(var c in this.cells){
            var pos = this.cells[c].hx_cell.Cell.pos;
            if (pos.y % 2 == 0){ //neighbor list for evens
                var neighbors = [
                    [pos.x-1,pos.y],[pos.x,pos.y-1],[pos.x-1,pos.y-1],
                    [pos.x+1,pos.y],[pos.x-1,pos.y+1],[pos.x,pos.y+1]
                ];
            }else{ //neighbor list for odds
                var neighbors = [
                    [pos.x-1,pos.y],[pos.x,pos.y-1],[pos.x+1,pos.y-1],
                    [pos.x+1,pos.y],[pos.x+1,pos.y+1],[pos.x,pos.y+1]
                ];
            }
            this.cells[c].hx_cell.neighbors = this.validateNeighbors(neighbors, false);
        }   
        
    }

    findNeighbor(pos, mark, returnType){
        var key = String(pos.x) + "," + String(pos.y)
        var cellObject = this.cells[key];
        var cell = cellObject.hx_cell.Cell;

        if (pos.y % 2 == 0){ //neighbor list for evens
            var neighbors = [
                [pos.x-1,pos.y],[pos.x,pos.y-1],[pos.x-1,pos.y-1],
                [pos.x+1,pos.y],[pos.x-1,pos.y+1],[pos.x,pos.y+1]
            ];
        }else{ //neighbor list for odds
            var neighbors = [
                [pos.x-1,pos.y],[pos.x,pos.y-1],[pos.x+1,pos.y-1],
                [pos.x+1,pos.y],[pos.x+1,pos.y+1],[pos.x,pos.y+1]
            ];
        }
        //console.log(this.validateNeighbors(neighbors));
        if (returnType == "object")
            return this.validateNeighbors(neighbors, mark);
        else if (returnType == "array")
            return neighbors;
    }

    validateNeighbors(arr, clicked){
        var newArr = []
        for(var i=0;i<arr.length;i++){
            var key = String(arr[i][0]+","+arr[i][1]);
            if (!(key in this.cells)){
                continue;
            }
            newArr[newArr.length] = this.cells[key]
            if (clicked){
                newArr[newArr.length-1].hx_tile.Tile.material.opacity = 0
                newArr[newArr.length-1].hx_tile.Tile.material.transparent = true;
                newArr[newArr.length-1].hx_tile.Tile.material.color.set( 0xFFFFFF );
            }
        }
        
        return newArr;
    }

    calculatePath(A,B, greedyWeight, dijkstrasWeight){
        //console.log(A)
        var key = String(A[0]) + "," + String(A[1])
        var PQ = new PriorityQueue();
        PQ.push(A,0);
        var came_from = {}
        var cost_so_far = {};
        came_from[key] = null;
        cost_so_far[key] = 0;

        while(PQ.empty() == false){
            var runner = PQ.pop();
            console.log(runner, B);

            if(runner.item == B)
                break;
            
            //var _key = String(runner.item[0]) + "," + String(runner.item[1])   
            //console.log(this.grid[_key].hx_tile.gamedata['edgeCost']);
            var neighbors = this.findNeighbor({'x':runner.item[0], 'y':runner.item[1]}, false, 'object')

            for(var n in neighbors){
                var n_key = String(neighbors[n].hx_cell.Cell.pos.x) + "," + String(neighbors[n].hx_cell.Cell.pos.y) 
                var new_cost = cost_so_far[key]  + this.nodeCost(key, n_key) 

                if (cost_so_far.hasOwnProperty(n_key) == false || new_cost < cost_so_far[n_key]){
                    cost_so_far[n_key] = new_cost;
                    var pos = [neighbors[n].hx_cell.Cell.pos.x, neighbors[n].hx_cell.Cell.pos.y]
                    var h = this.heuristic( this.calculateDistance(pos, B), greedyWeight, dijkstrasWeight, this.nodeCost(key, n_key));
                    var priority = new_cost + h;  //f(n) = g(n) + h(n)
                    //console.log(pos)
                    PQ.push(pos,priority);
                    came_from[n_key] = runner.item;
                }
            }
        }
        console.log(came_from);
        var path = this.reconstruct_path(came_from,A,B);
        console.log(path);
        return path;
    }

    reconstruct_path(came_from, A, B){
        var current = B
        var path = []
        while (current != A){
            path.push(current)
            current = came_from[current]
        }
        path.push(A)
        path.reverse()
        return path;
    }

    nodeCost(A,B){
        //console.log(A,B);
        return this.cells[A].hx_tile.gamedata.edgeCost + this.cells[B].hx_tile.gamedata.edgeCost;
    }

    heuristic(h,w1,w2,cost){
        //console.log(h,w1,w2,cost);
        h += h * w1;
        var c = cost * w2;
        return h + c;
    }

    calculateDistance(A,B){

        var posA = this.offset_to_cube(A);
        var posB = this.offset_to_cube(B);
        //console.log(posA)
        return this.findDistance(posA, posB)
    }

    cube_to_evenr(cords){
        var col = cube.x + (cube.z + (cube.z%2)) / 2
        var row = cube.z

        return [col, row]
    }

    offset_to_cube(cords){
        var x = cords[0] - (cords[1] + 1) / 2
        var z = cords[1]
        var y = -x-z
        return {
            'x':x,
            'y':y,
            'z':z
        }
    }

    findDistance(posA, posB){
        return (Math.abs(posA.x - posB.x) + Math.abs(posA.y - posB.y) + Math.abs(posA.z - posB.z)) / 2
    }

    heurustic(){
        return 1;
    }
}