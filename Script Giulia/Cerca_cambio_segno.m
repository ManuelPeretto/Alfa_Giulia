
function Posizione=Cerca_cambio_segno(Fg) 

% Cerco le intersezioni , cioè dove questo vettore cambia segno
segno=sign(Fg);
differenza=diff(segno);
Posizione=find(differenza);
   
      
     
