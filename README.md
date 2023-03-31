# Licenta2021 - Sistem de auto-organizare folosind roboți colaborativi

## Obiectivele proiectului:   

Inteligența colectivă este ramura științifica care studiază modul în care în natură apar sisteme biologice compuse din entități care la prima vedere par insignifiante, dar care colaboreaza și se comportă ca și cum ar fi un organism mult mai mare, îndeplinind sarcini care pentru ele individuale erau imposibile. Un bun exemplu ar fi furnicile sau albinele, ființe vulnerabile care dacă daca acestea ar trăi izolate ar avea o mică șansă de supraviețuire, însă când se organizeaza in grupuri sunt în stare să  adune hrana în mod eficient, să construiască adăposturi sigure și impresionante care să le ferească de condițiile externe și, în unele cazuri, chiar să atace pradatori mai mari. Acesta este și cazul sistemelor robotice, formate dintr-un numar mare de roboți autonomi cu un set limitat de capabilități dar care reușesc să comunice unii cu alții și să colaboreze pentru îndeplinirea unui scop comun. Această lucrare investighează și analizează algoritmi care se folosesc de acest comportament care apare în mod spontan în natură și îi implementează folosind un colectiv de roboți, numiți kilobots.  Kiloboții sunt roboți cu diametrul de 33 de mm, care se deplaseaza cu ajutorul a doua vibro motoare și comunică prin intermediul luminii infraroșii. Proiectul s-a axat în special pe algoritmi în care grupul de roboți se auto-organizează și crează diferite forme bidimensionale,  deplasându-se pe o cale fără blocaje și fără coliziuni. 
Realizarea proiectului și rezultate obținute:   Au fost investigate două tipuri de algoritmi în care grupul de roboți a primit sarcina de a forma figuri date de către un utilizator.  Prima metodă împarte imaginea într-o rețea de elemente echidistante și de aceași dimensiune și asociază fiecare element cu câte un robot. În cazul nostru, rețeaua este formată din cercuri de diametrul 33 mm aflate la distanțe de 50 mm unul de altul, această distanță putând fi modificată ulterior dacă se dorește ca roboții să fie mai dispersați. A doua metoda presupune că roboții își formează propriul sistem de coordonate pe care îl împărtășesc și îl folosesc pentru a crea formele dorite. Acești algoritmi s-au testat în cadrul unei simulări, folosind programul CoppeliaSim, dar și un grup real de kiloboți.  

![image](https://user-images.githubusercontent.com/86794414/229124038-f2b74e92-5312-40d0-95c6-1ade30ba031e.png)

În figura 1 sunt prezentate etapele primului tip de algoritim, implementat folosind colectivul de roboți.  Primul pas este trecerea imaginii dorite prin câțiva pași de procesare pentru a fi transformată într-o matrice care va putea fi trimisă roboților. Fiecare rand din matrice corespunde unui unui spațiu valid din formă, caracterizat prin distanța față de vecinii săi, de exemplu robotul de pe prima linie a matricii se va găsi la distanța de √2*  50 mm de robotul cu indexul 2 si de  1 * 50 mm de robotul numarul 3. Algoritmul presupune ca inițial, cel puțin doi roboți să se afle corect plasați în forma, pentru ca ceilalți să îi poată folosi pentru orientare. După cum se poate observa în figură, roboți incep așezați într-o linie dreaptă și, folosind un tip de algoritm de detectare a marginilor reușesc să decidă în ce ordine să pornească. Modul de deplasare al roboților este realizat prin implementarea unei proceduri de orbitare în jurul grupului, care garantează că nu vor exista coliziuni pe traseul acestora. În momentul în care un robot detectează că a ajuns pe o poziție liberă din formă acesta își oprește mișcarea și devine de asemenea punct de orientare pentru roboții care vor urma. Faptul că un kilobot a ocupat un spațiu din imagine este evidențiat prin aprinderea ledului într-o culoare mov. 

![image](https://user-images.githubusercontent.com/86794414/229124266-da7f623a-5694-47a2-8a30-18d726b7fb04.png)

A doua abordare, presupune că imaginea primită de la utilizator este o zonă delimitată de un poligon în care roboții vor trebui să se adune, renunțănd astfel la ideea că fiecare are un loc prestabilit în formă. Pornind de la conceptul triangularizării, care declară că se poate stabili poziția unui punct folosind cel puțin alte 3 puncte de referință, kiloboții vor comunica între ei și își vor crea propriul sistem de coordonate pe care îl vor folosi pentru a verifica dacă sunt în interiorul poligonului sau nu. Folosind aceiași algoritmi de orbitare și de detectare a marginilor ca în cadrul experimentului trecut, se poate observa in figura 2 că roboții reușesc să coopereze și să obțină o forma satisfăcătoare. Acest test a fost realizat în cadrul programului de simulare pentru robotică CoppeliaSim. Roboții reprezentați cu verde reprezintă originea sistemului, cei cu mov sunt cei care au considerat că s-au plasat în mod corect în formă și au încetat mișcarea iar albastru deschis este folosit pentru a evidenția roboții care realizează că sunt în interiorul poligonului dar care nu se vor opri din deplasare pană când nu vor ajunge la marginile poligonului sau pană când în calea lor nu vor întâlni alt robot. La începutul algoritmului kiloboții se află plasați într-o linie dreaptă și prin procesul de formare al gradientului  determina care dintre ei se află la marginea grupului și ar trebui să plece primul.

De asemenea, pentru a testa doar partea în care roboții verifică și decid dacă sunt în interior unui poligon oarecare s-a realizat urmatorul experiment, prezentat in figura 3. Treizeci și șase de roboți au fost plasați în interiorul unui patrat, toți cunoscându-și cu precizie poziția în rețeaua formată. Fiecare primește la începutul simulării un cuvănt, în cazul nostru “KILO’’ , îl împarte în litere și o dată la 6 secunde formează o literă nouă. Acest lucru este posibil deoarece în memoria kiloboților, fiecare literă este stocată sub forma unui poligon, iar folosind o adaptare a algoritmului de Ray-Casting, determină dacă se află in interiorul poligonului, caz in care își modifică culoarea in mov, sau nu, reprezentat de culoarea albastru deschis.

![image](https://user-images.githubusercontent.com/86794414/229124438-ab072d5c-1a19-4f9f-a940-6580864b4c94.png)



