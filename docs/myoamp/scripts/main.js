

let myButton = document.querySelector("button");
let myHeading = document.querySelector("h1");

window.addEventListener('load', () => {
  const img = document.querySelector('.slide-up');
  img.style.transform = 'rotate(-45deg) translateX(-450px) translateY(-450px)'; // final position
  img.style.opacity = '1';
});


  document.addEventListener("DOMContentLoaded", () => {
  const img = document.querySelector(".flow-img");
  if (!img) return;

  img.addEventListener("click", () => {
    img.classList.toggle("enlarged");
  });
});







