$(document).ready(function () {
    $('.image-form').on('submit', function (e) {
        e.preventDefault();
        console.log(e);
        console.log("form submitted");
    });
});

    // $('#image-input').on('change', function (event) {
    //     const file = event.target.files[0];
    //     if (file) {
    //         const reader = new FileReader();
    //         reader.onload = () => {
    //             $('#image-preview').attr('src', e.target.result);
    //
    //             const base64Image = reader.result;
    //             socket.emit('sendImage'), base64Image);
    //         };
    //     }
    // });
//
// function previewFiles() {
//   const preview = document.querySelector("#preview");
//   const files = document.querySelector("input[type=file]").files;
//
//   function readAndPreview(file) {
//     // Make sure `file.name` matches our extensions criteria
//     if (/\.(jpe?g|png|gif)$/i.test(file.name)) {
//       const reader = new FileReader();
//
//       reader.addEventListener(
//         "load",
//         () => {
//           const image = new Image();
//           image.height = 100;
//           image.title = file.name;
//           image.src = reader.result;
//           preview.appendChild(image);
//         },
//         false,
//       );
//
//       reader.readAsDataURL(file);
//     }
//   }
//
//   if (files) {
//     Array.prototype.forEach.call(files, readAndPreview);
//   }
// }
//
// const picker = document.querySelector("#browse");
// picker.addEventListener("change", previewFiles);
